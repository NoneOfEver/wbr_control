/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/chassismotors_send_raw.hpp>

#include "can_dispatch.h"

namespace {

constexpr uint8_t kBusCount = 4U;
constexpr uint16_t kLeftWheelId = 0x201;
constexpr uint16_t kRightWheelId = 0x202;
constexpr uint16_t kLeftJointBMasterId = 0x10U;
constexpr uint16_t kLeftJointDMasterId = 0x13U;
constexpr uint16_t kRightJointBMasterId = 0x11U;
constexpr uint16_t kRightJointDMasterId = 0x12U;
constexpr uint16_t kYawId = 0x05;
constexpr uint16_t kPitchId = 0x06;
constexpr uint16_t kPluckerId = 0x201;
constexpr uint16_t kLeftFricerId = 0x202;
constexpr uint16_t kRightFricerId = 0x203;
constexpr uint32_t kDefaultCanBitrate = 1000000U;
constexpr uint32_t kTxErrorLogPeriod = 100U;
constexpr uint32_t kRxUnmatchedLogPeriod = 1000U;

struct BusRxContext {
	uint8_t bus;
};

struct BusRxStats {
	uint32_t total = 0U;
	uint32_t routed = 0U;
	uint32_t unmatched = 0U;
};

const struct device *g_can_dev[kBusCount] = {nullptr, nullptr, nullptr, nullptr};
bool g_started = false;
bool g_tx_thread_started = false;
k_thread g_tx_thread;
K_THREAD_STACK_DEFINE(g_can_tx_stack, 1024);
uint32_t g_tx_enqueue_error_count = 0U;
uint32_t g_tx_callback_error_count = 0U;
BusRxContext g_rx_context[kBusCount] = {
	{0U},
	{1U},
	{2U},
	{3U},
};
BusRxStats g_rx_stats[kBusCount] = {};

uint32_t ConfiguredBitrateForBus(uint8_t bus)
{
	if (bus == 0U) {
		return DT_PROP_OR(DT_NODELABEL(can0), bitrate, kDefaultCanBitrate);
	}
	if (bus == 1U) {
		return DT_PROP_OR(DT_NODELABEL(can1), bitrate, kDefaultCanBitrate);
	}
	if (bus == 2U) {
		return DT_PROP_OR(DT_NODELABEL(can2), bitrate, kDefaultCanBitrate);
	}
	if (bus == 3U) {
		return DT_PROP_OR(DT_NODELABEL(can3), bitrate, kDefaultCanBitrate);
	}
	return kDefaultCanBitrate;
}

const char *CanStateToString(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

void PrintCanState(const struct device *dev, uint8_t bus, const char *tag)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	const int rc = can_get_state(dev, &state, &err_cnt);
	if (rc == 0) {
		printk("[can_dispatch] bus%u %s: %s tec=%u rec=%u\n",
		       static_cast<unsigned int>(bus), tag, CanStateToString(state),
		       static_cast<unsigned int>(err_cnt.tx_err_cnt),
		       static_cast<unsigned int>(err_cnt.rx_err_cnt));
	} else {
		printk("[can_dispatch] bus%u %s: can_get_state rc=%d\n",
		       static_cast<unsigned int>(bus), tag, rc);
	}
}

const struct device *FindCanDeviceForBus(uint8_t bus)
{
	if (bus == 0U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can0));
#else
		return nullptr;
#endif
	}
	if (bus == 1U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can1));
#else
		return nullptr;
#endif
	}
	if (bus == 2U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can2));
#else
		return nullptr;
#endif
	}
	if (bus == 3U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can3));
#else
		return nullptr;
#endif
	}

	return nullptr;
}

void OnTxDone(const struct device *dev, int error, void *user_data)
{
	if (error == 0) {
		return;
	}

	const uint32_t packed = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(user_data));
	const uint8_t bus = static_cast<uint8_t>((packed >> 16U) & 0xFFU);
	const uint16_t can_id = static_cast<uint16_t>(packed & 0xFFFFU);
	const uint32_t count = ++g_tx_callback_error_count;
	if ((count <= 10U) || ((count % kTxErrorLogPeriod) == 0U)) {
		printk("[can_dispatch] tx callback error bus%u id=0x%x error=%d count=%u\n",
		       static_cast<unsigned int>(bus), static_cast<unsigned int>(can_id),
		       error, static_cast<unsigned int>(count));
		PrintCanState(dev, bus, "tx callback error");
	}
}

int SendStdFrame(uint8_t bus, uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
	if ((bus >= kBusCount) || (data == nullptr) || (dlc > 8U)) {
		return -EINVAL;
	}

	const struct device *dev = g_can_dev[bus];
	if (dev == nullptr) {
		return -ENODEV;
	}

	struct can_frame frame = {};
	frame.flags = 0U;
	frame.id = can_id;
	frame.dlc = dlc;
	for (uint8_t i = 0U; i < dlc; ++i) {
		frame.data[i] = data[i];
	}

	const uint32_t packed_user_data =
		(static_cast<uint32_t>(bus) << 16U) | static_cast<uint32_t>(can_id);
	const int rc = can_send(dev, &frame, K_MSEC(2), OnTxDone,
				reinterpret_cast<void *>(static_cast<uintptr_t>(packed_user_data)));
	if (rc != 0) {
		const uint32_t count = ++g_tx_enqueue_error_count;
		if ((count <= 10U) || ((count % kTxErrorLogPeriod) == 0U)) {
			printk("[can_dispatch] can_send enqueue failed bus%u id=0x%x rc=%d count=%u\n",
			       static_cast<unsigned int>(bus), static_cast<unsigned int>(can_id),
			       rc, static_cast<unsigned int>(count));
			PrintCanState(dev, bus, "enqueue failed");
		}
	}
	return rc;
}

void CanTxLoop(void *, void *, void *)
{
	SeqlockValue<ChassisMotorSendRawFrame> *const slots[] = {
		&left_wheel_send_raw,
		&right_wheel_send_raw,
		&left_B_motor_send_raw,
		&left_D_motor_send_raw,
		&right_B_motor_send_raw,
		&right_D_motor_send_raw,
	};
	uint32_t last_sequence[sizeof(slots) / sizeof(slots[0])] = {};

	for (;;) {
		for (size_t i = 0U; i < (sizeof(slots) / sizeof(slots[0])); ++i) {
			const uint32_t sequence = slots[i]->sequence();
			if ((sequence == 0U) || (sequence == last_sequence[i])) {
				continue;
			}

			ChassisMotorSendRawFrame local_frame = {};
			if (!slots[i]->read(local_frame)) {
				continue;
			}
			last_sequence[i] = sequence;
			(void)SendStdFrame(local_frame.bus, local_frame.can_id, local_frame.data, local_frame.dlc);
		}
		k_sleep(K_MSEC(1));
	}
}


void CanRxCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);

	if ((frame == nullptr) || ((frame->flags & CAN_FRAME_IDE) != 0U)) {
		return;
	}

	const BusRxContext *context = static_cast<const BusRxContext *>(user_data);
	if ((context == nullptr) || (context->bus >= kBusCount)) {
		return;
	}
	const uint8_t bus = context->bus;
	BusRxStats &stats = g_rx_stats[bus];
	++stats.total;

	ChassisMotorFeedbackRawFrame rx_frame = {};
	for (uint8_t i = 0U; (i < frame->dlc) && (i < sizeof(rx_frame.data)); ++i) {
		rx_frame.data[i] = frame->data[i];
	}

	bool routed = false;
	if (bus == 0U) {
		switch (static_cast<uint16_t>(frame->id)) {
		case kLeftWheelId:
			left_wheel_feedback_raw.write(rx_frame);
			routed = true;
			break;
		case kLeftJointBMasterId:
			left_B_motor_feedback_raw.write(rx_frame);
			routed = true;
			break;
		case kLeftJointDMasterId:
			left_D_motor_feedback_raw.write(rx_frame);
			routed = true;
			break;
		default:
			break;
		}
	} else if (bus == 1U) {
		switch (static_cast<uint16_t>(frame->id)) {
		case kRightWheelId:
			right_wheel_feedback_raw.write(rx_frame);
			routed = true;
			break;
		case kRightJointBMasterId:
			right_B_motor_feedback_raw.write(rx_frame);
			routed = true;
			break;
		case kRightJointDMasterId:
			right_D_motor_feedback_raw.write(rx_frame);
			routed = true;
			break;
		default:
			break;
		}
	} else if (bus == 2U) {

	} else if (bus == 3U) {

	} else {
		return;
	}

	if (routed) {
		++stats.routed;
		return;
	}

	const uint32_t unmatched = ++stats.unmatched;
	if ((unmatched <= 20U) || ((unmatched % kRxUnmatchedLogPeriod) == 0U)) {
		printk("[can_dispatch] rx unmatched bus%u id=0x%x dlc=%u total=%u unmatched=%u\n",
		       static_cast<unsigned int>(bus), static_cast<unsigned int>(frame->id),
		       static_cast<unsigned int>(frame->dlc),
		       static_cast<unsigned int>(stats.total),
		       static_cast<unsigned int>(unmatched));
	}
}

int AddDefaultFilters(const struct device *dev, uint8_t bus)
{
	if ((dev == nullptr) || (bus >= kBusCount)) {
		return -EINVAL;
	}

	const struct can_filter all_standard_frames = {
		.id = 0U,
		.mask = 0U,
		.flags = 0U,
	};

	const int filter_id = can_add_rx_filter(dev, CanRxCallback, &g_rx_context[bus],
						&all_standard_frames);
	if (filter_id < 0) {
		printk("[can_dispatch] bus%u add catch-all rx filter failed: %d\n",
		       static_cast<unsigned int>(bus), filter_id);
		return filter_id;
	}

	printk("[can_dispatch] bus%u catch-all rx filter installed id=%d\n",
	       static_cast<unsigned int>(bus), filter_id);
	return 0;
}

}  // namespace

namespace platform::drivers::communication::can_dispatch {

int Initialize()
{
	if (g_started) { 
		return 0; 
	}

	bool has_any_bus = false;
	for (uint8_t bus = 0U; bus < kBusCount; ++bus) {
		g_can_dev[bus] = FindCanDeviceForBus(bus);
		if ((g_can_dev[bus] == nullptr) || !device_is_ready(g_can_dev[bus])) {
			printk("[can_dispatch] bus%u device not ready\n",
			       static_cast<unsigned int>(bus));
			continue;
		}

		has_any_bus = true;

		const int mode_rc = can_set_mode(g_can_dev[bus], CAN_MODE_NORMAL);
		if (mode_rc != 0) {
			printk("[can_dispatch] bus%u can_set_mode(normal) failed: %d\n",
			       static_cast<unsigned int>(bus), mode_rc);
			return mode_rc;
		}

		const uint32_t bitrate = ConfiguredBitrateForBus(bus);
		const int bitrate_rc = can_set_bitrate(g_can_dev[bus], bitrate);
		if (bitrate_rc != 0) {
			printk("[can_dispatch] bus%u can_set_bitrate(%u) failed: %d\n",
			       static_cast<unsigned int>(bus), static_cast<unsigned int>(bitrate),
			       bitrate_rc);
			return bitrate_rc;
		}

		const int rc = can_start(g_can_dev[bus]);
		if ((rc != 0) && (rc != -EALREADY)) {
			printk("[can_dispatch] bus%u can_start failed: %d\n",
			       static_cast<unsigned int>(bus), rc);
			return rc;
		}

		printk("[can_dispatch] bus%u started dev=%s bitrate=%u\n",
		       static_cast<unsigned int>(bus), g_can_dev[bus]->name,
		       static_cast<unsigned int>(bitrate));
		PrintCanState(g_can_dev[bus], bus, "after start");

		const int filter_rc = AddDefaultFilters(g_can_dev[bus], bus);
		if (filter_rc != 0) {
			return filter_rc;
		}
	}

	if (!has_any_bus) {
		return -ENODEV;
	}

	if (!g_tx_thread_started) {
		k_tid_t tid = k_thread_create(&g_tx_thread,
					      g_can_tx_stack,
					      K_THREAD_STACK_SIZEOF(g_can_tx_stack),
					      CanTxLoop,
					      nullptr,
					      nullptr,
					      nullptr,
					      K_PRIO_PREEMPT(7),
					      0,
					      K_NO_WAIT);
		k_thread_name_set(tid, "can_tx");
		g_tx_thread_started = true;
	}

	g_started = true;
	return 0;
}

}  // namespace platform::drivers::communication::can_dispatch
