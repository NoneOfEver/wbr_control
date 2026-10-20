/* SPDX-License-Identifier: Apache-2.0 */
#include "peripherals.h"
#include <protocols/usb_bridge/fault_diagnostics.hpp>
#include <chassis_controller/communication/can_dispatch.h>
#include <chassis_controller/communication/usb_session.h>
#include <chassis_controller/motor_map.h>
#include <protocols/motors/dm_motor_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <cstring>

namespace
{
namespace wire = wbr::bridge;
using wire::Packet;
constexpr unsigned kBusMotors[2][3] = {{0, 2, 3}, {1, 4, 5}}; // W/B/D
K_MSGQ_DEFINE(peripheral_packets, sizeof(Packet), 128, 4);
K_MSGQ_DEFINE(session_replies, sizeof(Packet), 4, 4);
K_SEM_DEFINE(uplink_event, 0, 1);
K_THREAD_STACK_DEFINE(uplink_stack, 2048);
k_thread uplink_thread;
atomic_t session_id;
Packet latest_can[6]{};
uint8_t dirty_can;
wire::CommandLease command_lease;
uint32_t capabilities;
uint32_t NowUs()
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
wire::FaultSnapshot fault_stats;
#endif
bool SafeStop()
{
	bool submitted = true;
	command_lease.Reset();
	// Physical shutdown policy remains local even if the host/USB disappears.
	uint8_t exit[8]{};
	protocols::GetDmControlCommandFrame(protocols::DmControlCommand::kExit, exit);
	for (unsigned group = 0; group < 2; ++group) {
		platform::CanStandardFrame frames[3]{};
		for (unsigned i = 0; i < 3; ++i) {
			const auto &motor = platform::motor_map::kMotors[kBusMotors[group][i]];
			frames[i].slot = motor.tx_slot;
			frames[i].can_id = motor.command_id;
			frames[i].dlc = 8;
			if (i)
				std::memcpy(frames[i].data, exit,
					    8); // wheel group defaults to zero current
		}
		if (platform::SubmitCanStandardFrames(group == 0 ? 3 : 2, frames, 3) != 0)
			submitted = false;
	}
	return submitted;
}

void PolicyStop(wire::StopReason reason)
{
	const bool submitted = SafeStop();
#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
	fault_stats.last_stop_submit_us = NowUs();
	fault_stats.stop_flags = uint32_t(reason) | (submitted ? 0U : wire::kStopSubmissionError);
	if (reason == wire::StopReason::Explicit) ++fault_stats.explicit_stops;
	if (reason == wire::StopReason::LeaseExpired || reason == wire::StopReason::UsbDeconfigured)
		++fault_stats.lease_stops;
#else
	(void)submitted;
	(void)reason;
#endif
}

void ReceiveCan(const platform::CanReceiveFrame &frame, void *)
{
	if (!atomic_get(&session_id))
		return;
	// Configured motor streams are latest-state mailboxes, never an unbounded FIFO.
	for (unsigned i = 0; i < 6; ++i) {
		const auto &motor = platform::motor_map::kMotors[i];
		if (motor.bus != frame.bus || motor.feedback_id != frame.can_id)
			continue;
		const unsigned key = irq_lock();
		wire::CanRecord(latest_can[i].data(), frame.bus, frame.can_id, frame.data,
				frame.dlc, frame.timestamp_us);
		dirty_can |= uint8_t(1U << i);
		irq_unlock(key);
		k_sem_give(&uplink_event);
		return;
	}
	Packet packet{};
	wire::Header(packet.data(), wire::Kind::Can, 0, gateway::Session());
	packet[10] = 1;
	wire::CanRecord(packet.data() + 16, frame.bus, frame.can_id, frame.data, frame.dlc,
			frame.timestamp_us);
	gateway::PublishPeripheral(packet);
}

bool TakeCan(Packet &packet)
{
	const unsigned key = irq_lock();
	unsigned count = 0;
	static unsigned next = 0;
	for (unsigned visited = 0; visited < 6 && count < 3; ++visited) {
		unsigned motor = next;
		next = (next + 1) % 6;
		if (!(dirty_can & (1U << motor)))
			continue;
		std::memcpy(packet.data() + 16 + 16 * count, latest_can[motor].data(), 16);
		dirty_can &= uint8_t(~(1U << motor));
		++count;
	}
	irq_unlock(key);
	if (!count)
		return false;
	wire::Header(packet.data(), wire::Kind::Can, NowUs(), gateway::Session());
	packet[10] = count;
	return true;
}

void Uplink(void *, void *, void *)
{
	bool peripheral_turn = false;
	for (;;) {
		Packet packet{};
		bool found = k_msgq_get(&session_replies, &packet, K_NO_WAIT) == 0;
		if (!found && peripheral_turn)
			found = k_msgq_get(&peripheral_packets, &packet, K_NO_WAIT) == 0;
		if (!found)
			found = TakeCan(packet);
		if (!found)
			found = k_msgq_get(&peripheral_packets, &packet, K_NO_WAIT) == 0;
		if (!found) {
			k_sem_take(&uplink_event, K_FOREVER);
			continue;
		}
		peripheral_turn = !peripheral_turn;
		if (wire::U32(packet.data() + 12) != gateway::Session())
			continue;
		// Refresh only the send timestamp, retaining acquisition timestamps in records.
		wire::Put32(packet.data() + 4, NowUs());
		if (platform::SendUsb(packet.data(), packet.size()) == int(packet.size()))
			(void)platform::WaitUsbTxComplete(20);
	}
}

bool Forward(const wire::Commands &packet)
{
	if (!wire::ValidCommands(packet, gateway::Session()))
		return false;
	const uint32_t sequence = wire::U32(packet.data() + 4);
	if (!command_lease.CanAccept(sequence))
		return false;
	// Validate every frame before publishing either bus. Never partly apply an invalid packet.
	const uint8_t *records[6]{};
	for (unsigned i = 0; i < packet[10]; ++i) {
		const auto *record = packet.data() + 32 + 16 * i;
		const auto &motor = platform::motor_map::kMotors[record[12]];
		if (record[0] != motor.bus || wire::U16(record + 2) != motor.command_id)
			return false;
		records[record[12]] = record;
	}
	bool success = true;
	k_sched_lock();
	for (unsigned group = 0; group < 2; ++group) {
		platform::CanStandardFrame frames[3]{};
		unsigned count = 0;
		for (auto motor_index : kBusMotors[group]) {
			const auto *record = records[motor_index];
			if (!record)
				continue;
			auto &frame = frames[count++];
			frame.slot = platform::motor_map::kMotors[motor_index].tx_slot;
			frame.can_id = wire::U16(record + 2);
			frame.dlc = 8;
			std::memcpy(frame.data, record + 4, 8);
		}
		if (count &&
		    platform::SubmitCanStandardFrames(group == 0 ? 3 : 2, frames, count) != 0)
			success = false;
	}
	k_sched_unlock();
	if (!success) {
		PolicyStop(wire::StopReason::CanSubmissionFailed);
		atomic_clear(&session_id);
		return false;
	}
	const uint32_t accepted_us = NowUs();
	command_lease.Renew(sequence, accepted_us);
#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
	++fault_stats.accepted;
	fault_stats.last_accepted_us = accepted_us;
#endif
	return true;
}
} // namespace

namespace gateway
{
uint32_t Session()
{
	return static_cast<uint32_t>(atomic_get(&session_id));
}
void PublishPeripheral(const Packet &packet)
{
	if (!Session())
		return;
	// Stream sequence numbers allow host parsers to reset if this bounded queue drops a chunk.
	if (k_msgq_put(&peripheral_packets, &packet, K_NO_WAIT) == 0)
		k_sem_give(&uplink_event);
}
} // namespace gateway

int main()
{
	k_thread_priority_set(k_current_get(), 7);
	platform::SetCanReceiveCallback(ReceiveCan, nullptr);
	if (platform::InitializeCanDispatch(false, true) != 0)
		return -1;
	SafeStop();
	capabilities = 1 | gateway::StartPeripherals();
	if (platform::InitializeUsbSession() != 0)
		return -1;
	k_thread_create(&uplink_thread, uplink_stack, K_THREAD_STACK_SIZEOF(uplink_stack), Uplink,
			nullptr, nullptr, nullptr, 7, 0, K_NO_WAIT);
	for (;;) {
		wire::Commands packet{};
		size_t length = 0;
		if (platform::ReceiveUsb(packet.data(), packet.size(), &length, 1) == 0) {
			if (length == 64 &&
			    wire::HasHeader(packet.data(), length, wire::Kind::Hello)) {
#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
				Packet fault_reply{};
				const bool fault_query = packet[10] == wire::kFaultQueryPage;
				if (fault_query) {
					fault_stats.prior_session = gateway::Session();
					wire::EncodeFaultSnapshot(fault_reply, fault_stats);
				}
#endif
				SafeStop();
				const uint32_t requested = wire::U32(packet.data() + 12);
				const unsigned key = irq_lock();
				atomic_set(&session_id, requested);
				dirty_can = 0;
				k_msgq_purge(&peripheral_packets);
				k_msgq_purge(&session_replies);
				irq_unlock(key);
				if (requested) {
					Packet reply{};
					wire::Header(reply.data(), wire::Kind::Ready, NowUs(),
						     requested);
					wire::Put32(reply.data() + 16, capabilities);
					wire::Put32(reply.data() + 20, wire::kOutBytes);
					wire::Put32(reply.data() + 24, wire::kInBytes);
#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
					if (fault_query) std::memcpy(reply.data()+28, fault_reply.data()+28, 36);
					else
#endif
					gateway::AppendUartProbeStatus(reply);
					k_msgq_put(&session_replies, &reply, K_NO_WAIT);
					k_sem_give(&uplink_event);
				}
			} else if (length == 128) {
				const bool accepted = gateway::Session() && Forward(packet);
#ifdef CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS
				if (!accepted) ++fault_stats.rejected;
#else
				(void)accepted;
#endif
			} else if (length == 64 &&
				   wire::HasHeader(packet.data(), length, wire::Kind::Stop) &&
				   wire::U32(packet.data() + 12) == gateway::Session()) {
				PolicyStop(wire::StopReason::Explicit);
				atomic_clear(&session_id);
			}
		}
		const bool configured = platform::IsUsbConfigured();
		if (command_lease.Expired(NowUs(), configured)) {
			PolicyStop(configured ? wire::StopReason::LeaseExpired : wire::StopReason::UsbDeconfigured);
			atomic_clear(&session_id); // new handshake required to resume
		}
	}
}
