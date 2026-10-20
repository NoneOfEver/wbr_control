/* SPDX-License-Identifier: Apache-2.0 */
#include "peripherals.h"
#include <chassis_controller/modules/ahrs/onboard_imu_module.h>
#include <drivers/uart_hpmicro.h>
#include <hpm_l1c_drv.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <algorithm>
#include <cstring>

#ifdef CONFIG_WBR_UART10_PLIC_PROBE
extern "C" uint32_t wbr_uart_rx_probe_version(void);
#endif
namespace
{
namespace wire = wbr::bridge;
constexpr size_t kChunkBytes = 40, kBufferBytes = 128, kRingBytes = 4096;
__attribute__((section(".nocache"), aligned(64))) uint8_t hi91_ring[kRingBytes];
__attribute__((aligned(64))) uint8_t uart_buffers[2][2][kBufferBytes];
K_THREAD_STACK_DEFINE(uart_stack, 2048);
K_THREAD_STACK_DEFINE(imu_stack, 2048);
k_thread uart_thread, imu_thread;
modules::OnboardImu onboard;
struct UartPort {
	const device *dev = nullptr;
	wire::Port port{};
	unsigned buffer_set = 0, next_buffer = 1;
	uint32_t sequence = 0;
	void Publish(const uint8_t *data, size_t size)
	{
		while (size) {
			const size_t count = std::min(size, kChunkBytes);
			wire::Packet packet{};
			const uint32_t now = k_cyc_to_us_floor64(k_cycle_get_64());
			wire::Header(packet.data(), wire::Kind::Uart, now, gateway::Session());
			packet[10] = uint8_t(port);
			packet[11] = count;
			wire::Put32(packet.data() + 16, sequence++);
			wire::Put32(packet.data() + 20, now);
			std::memcpy(packet.data() + 24, data, count);
			gateway::PublishPeripheral(packet);
			data += count;
			size -= count;
		}
	}
};
#ifdef CONFIG_WBR_UART10_PLIC_PROBE
struct UartProbe {
	uint32_t irq_count = 0, ready_count = 0, partial_count = 0;
	uint32_t full_count = 0, bytes = 0, max_gap_us = 0, last_us = 0, last_length = 0;
	bool seen = false;
	uint32_t rx_available = 0, rx_timeout = 0, other_irq = 0, last_irq = 0;
	uint32_t timeout_flush = 0, zero_flush = 0, status_errors = 0;
	uint32_t total = 0, offset = 0, pending = 0, added = 0;
} probe;
void ProbeIrq(const device *, void *) { ++probe.irq_count; }
#endif
UartPort remote{DEVICE_DT_GET(DT_CHOSEN(wbr_control_remote_input_uart)), wire::Port::Remote, 0};
UartPort hi91{DEVICE_DT_GET(DT_CHOSEN(wbr_control_hi91_uart)), wire::Port::Hi91, 0};
#if DT_HAS_CHOSEN(wbr_control_referee_uart)
UartPort referee{DEVICE_DT_GET(DT_CHOSEN(wbr_control_referee_uart)), wire::Port::Referee, 1};
#endif

void AsyncUart(const device *dev, uart_event *event, void *user)
{
	auto &port = *static_cast<UartPort *>(user);
	switch (event->type) {
	case UART_RX_RDY: {
#ifdef CONFIG_WBR_UART10_PLIC_PROBE
		if (port.port == wire::Port::Remote) {
			const uint32_t now = k_cyc_to_us_floor64(k_cycle_get_64());
			++probe.ready_count;
			probe.partial_count += event->data.rx.len < kBufferBytes;
			probe.full_count += event->data.rx.len == kBufferBytes;
			probe.bytes += event->data.rx.len;
			if (probe.seen) probe.max_gap_us = std::max(probe.max_gap_us, now - probe.last_us);
			probe.last_us = now;
			probe.last_length = event->data.rx.len;
			probe.seen = true;
		}
#endif
		const auto *data = event->data.rx.buf + event->data.rx.offset;
		const uint32_t begin =
			HPM_L1C_CACHELINE_ALIGN_DOWN(reinterpret_cast<uint32_t>(data));
		const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(reinterpret_cast<uint32_t>(data) +
								event->data.rx.len);
		l1c_dc_invalidate(begin, end - begin);
		port.Publish(data, event->data.rx.len);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		const unsigned next = port.next_buffer;
		port.next_buffer ^= 1;
		(void)uart_rx_buf_rsp(dev, uart_buffers[port.buffer_set][next], kBufferBytes);
		break;
	}
	case UART_RX_DISABLED:
		port.next_buffer = 1;
		(void)uart_rx_enable(dev, uart_buffers[port.buffer_set][0], kBufferBytes, 1000);
		break;
	case UART_RX_STOPPED:
		// Advance stream sequence even if no chunk is emitted. Host discards a
		// partially assembled frame after an acquisition discontinuity.
		++port.sequence;
		break;
	default:
		break;
	}
}

bool StartAsync(UartPort &port)
{
	return device_is_ready(port.dev) && uart_callback_set(port.dev, AsyncUart, &port) == 0 &&
	       uart_rx_enable(port.dev, uart_buffers[port.buffer_set][0], kBufferBytes, 1000) == 0;
}
void Hi91Loop(void *, void *, void *)
{
	size_t read = 0, observed = 0, available = 0;
	uint32_t previous_poll = k_uptime_get_32();
	for (;;) {
		size_t position = 0;
		const uint32_t now = k_uptime_get_32();
		if (now - previous_poll > 20) {
			// A whole ring can be overwritten after a prolonged scheduling
			// stall; do not forward mixed old/new bytes as a valid stream.
			++hi91.sequence;
			if (uart_hpm_rx_circular_get_position(hi91.dev, &position) == 0)
				read = observed = position % kRingBytes;
			available = 0;
		}
		previous_poll = now;
		if (uart_hpm_rx_circular_get_position(hi91.dev, &position) == 0 &&
		    position <= kRingBytes) {
			position %= kRingBytes;
			available += (position + kRingBytes - observed) % kRingBytes;
			observed = position;
			// Retain the proven DMA outstanding-write guard from production.
			size_t consume = available > 16 ? available - 16 : 0;
			while (consume) {
				const size_t count = std::min(consume, kRingBytes - read);
				hi91.Publish(hi91_ring + read, count);
				read = (read + count) % kRingBytes;
				available -= count;
				consume -= count;
			}
		}
		k_sleep(K_MSEC(1));
	}
}
void ImuLoop(void *, void *, void *)
{
	for (;;) {
		modules::OnboardImu::Burst burst{};
		if (onboard.TryTakeCompleted(burst)) {
			wire::Packet packet{};
			const uint32_t now = k_cyc_to_us_floor64(k_cycle_get_64());
			wire::Header(packet.data(), wire::Kind::Imu, now, gateway::Session());
			packet[10] = modules::OnboardImu::kBurstTransferSize;
			const uint32_t age =
				k_cyc_to_us_floor32(k_cycle_get_32() - burst.data_ready_cycle);
			wire::Put32(packet.data() + 16, now - age);
			std::memcpy(packet.data() + 20, burst.rx, sizeof(burst.rx));
			std::memcpy(packet.data() + 48, &burst.temperature_c, sizeof(float));
			gateway::PublishPeripheral(packet);
		}
		(void)onboard.TryStartAsync();
		k_usleep(100);
	}
}
} // namespace

namespace gateway
{
void AppendUartProbeStatus(wire::Packet &packet)
{
#ifdef CONFIG_WBR_UART10_PLIC_PROBE
	const unsigned key = irq_lock();
	wire::Put32(packet.data() + 28, wbr_uart_rx_probe_version()); // URT3: driver-path instrumentation
	const uint32_t flags = 1U |
		(irq_is_enabled(DT_IRQN(DT_CHOSEN(wbr_control_remote_input_uart))) ? 2U : 0U) |
		(probe.added > 65535U ? 4U : 0U) |
		((probe.last_irq & 0xffU) << 8) | (std::min(probe.status_errors, 65535U) << 16);
	wire::Put32(packet.data() + 32, flags);
	wire::Put32(packet.data() + 36, probe.rx_available);
	wire::Put32(packet.data() + 40, probe.rx_timeout);
	wire::Put32(packet.data() + 44, probe.other_irq);
	wire::Put32(packet.data() + 48, probe.timeout_flush);
	wire::Put32(packet.data() + 52, probe.zero_flush);
	wire::Put32(packet.data() + 56, (probe.total & 0xffffU) | ((probe.offset & 0xffffU) << 16));
	wire::Put32(packet.data() + 60, (probe.pending & 0xffffU) | ((probe.added & 0xffffU) << 16));
	irq_unlock(key);
#else
	(void)packet;
#endif
}
uint32_t StartPeripherals()
{
	uint32_t capabilities = 0;
	if (StartAsync(remote)) {
		capabilities |= 2;
#ifdef CONFIG_WBR_UART10_PLIC_PROBE
		// A/B probe only: the HPM async init leaves this PLIC source masked,
		// while uart_rx_enable() enables only the UART peripheral IRQ. Keep
		// DMA buffers and timeout handling unchanged to isolate this gate.
		uart_irq_callback_user_data_set(remote.dev, ProbeIrq, nullptr);
		irq_enable(DT_IRQN(DT_CHOSEN(wbr_control_remote_input_uart)));
#endif
	}
	if (device_is_ready(hi91.dev) &&
	    uart_hpm_rx_circular_enable(hi91.dev, hi91_ring, kRingBytes) == 0) {
		capabilities |= 4;
		k_thread_create(&uart_thread, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack),
				Hi91Loop, nullptr, nullptr, nullptr, 7, 0, K_NO_WAIT);
	}
#if DT_HAS_CHOSEN(wbr_control_referee_uart)
	if (StartAsync(referee))
		capabilities |= 16;
#endif
	if (onboard.Init() == 0) {
		capabilities |= 8;
		k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), ImuLoop,
				nullptr, nullptr, nullptr, 4, 0, K_NO_WAIT);
	}
	return capabilities;
}
} // namespace gateway

#ifdef CONFIG_WBR_UART10_PLIC_PROBE
// Called at the driver's existing IIR read and timeout-only flush points.
// Event numbers are documented with diagnostics/hpm_uart_rx_probe.patch.
extern "C" void wbr_uart_rx_probe(const device *dev, uint32_t event,
                                  uint32_t a, uint32_t b, uint32_t c)
{
	if (dev != remote.dev) return;
	switch (event) {
	case 1:
		probe.last_irq = a;
		if (a == 0x4) ++probe.rx_available;
		else if (a == 0xc) ++probe.rx_timeout;
		else ++probe.other_irq;
		break;
	case 2: ++probe.timeout_flush; break;
	case 3: probe.total = a; probe.offset = b; probe.pending = c; break;
	case 4:
		if (b) ++probe.status_errors;
		else { probe.added = a; if (!a) ++probe.zero_flush; }
		break;
	default: break;
	}
}
#endif
