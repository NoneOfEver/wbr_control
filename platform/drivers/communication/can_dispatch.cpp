/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <app/channels/can_raw_frame_queue.h>

#include "can_dispatch.h"

namespace {

constexpr int kThreadPrio = 8;
constexpr uint8_t kBusCount = 3U;
constexpr uint16_t kDjiId1 = 0x201;
constexpr uint16_t kDjiId2 = 0x202;
constexpr uint16_t kDjiId3 = 0x203;
constexpr uint16_t kDjiId4 = 0x204;
constexpr uint16_t kDmId1 = 0x011;
constexpr uint16_t kDmId2 = 0x012;
constexpr uint16_t kDmId3 = 0x013;
constexpr uint16_t kCubemarsBroadcastId = 0x000;

struct CanRxEvent {
	uint8_t bus;
	uint16_t can_id;
	uint8_t dlc;
	uint8_t data[8];
};

K_THREAD_STACK_DEFINE(g_can_dispatch_stack, 1024);
K_MSGQ_DEFINE(g_can_rx_msgq, sizeof(CanRxEvent), 16, 4);

struct k_thread g_can_dispatch_thread;
const struct device *g_can_dev[kBusCount] = {nullptr, nullptr, nullptr};
bool g_started = false;

int BusToIndex(rm_test::platform::drivers::communication::can_dispatch::CanBus bus)
{
	const uint8_t raw = static_cast<uint8_t>(bus);
	if ((raw < 1U) || (raw > kBusCount)) {
		return -1;
	}

	return static_cast<int>(raw - 1U);
}

const struct device *FindCanDeviceForBus(uint8_t bus)
{
	const char *const can1_candidates[] = {"CAN_1", "can1", "MCAN1", "mcan1"};
	const char *const can2_candidates[] = {"CAN_2", "can2", "MCAN2", "mcan2"};
	const char *const can3_candidates[] = {
		"CAN_3", "can3", "MCAN3", "mcan3", "CAN_0", "can0", "CAN0", "MCAN4", "mcan4"};

	const char *const *candidates = nullptr;
	size_t candidate_count = 0U;
	if (bus == 1U) {
		candidates = can1_candidates;
		candidate_count = sizeof(can1_candidates) / sizeof(can1_candidates[0]);
	} else if (bus == 2U) {
		candidates = can2_candidates;
		candidate_count = sizeof(can2_candidates) / sizeof(can2_candidates[0]);
	} else if (bus == 3U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcan4), okay)
		const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mcan4));
		if (device_is_ready(dev)) {
			return dev;
		}
#endif
		candidates = can3_candidates;
		candidate_count = sizeof(can3_candidates) / sizeof(can3_candidates[0]);
	}

	for (size_t i = 0U; i < candidate_count; ++i) {
		const struct device *dev = device_get_binding(candidates[i]);
		if ((dev != nullptr) && device_is_ready(dev)) {
			return dev;
		}
	}

	return nullptr;
}

void RouteCanFrameToModuleQueues(const CanRxEvent &event)
{
	rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage frame = {};
	frame.bus = event.bus;
	frame.can_id = event.can_id;
	frame.dlc = event.dlc;
	for (uint8_t i = 0U; i < 8U; ++i) {
		frame.data[i] = event.data[i];
	}

	if (event.bus == 1U) {
		if ((event.can_id >= kDjiId1) && (event.can_id <= kDjiId4)) {
			(void)rm_test::app::channels::can_raw_frame_queue::EnqueueForChassis(&frame);
		}
		if (event.can_id == kCubemarsBroadcastId) {
			(void)rm_test::app::channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
		return;
	}

	if (event.bus == 2U) {
		if ((event.can_id == kCubemarsBroadcastId) || (event.can_id == 0x201U) ||
		    (event.can_id == 0x202U)) {
			(void)rm_test::app::channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
		return;
	}

	if (event.bus == 3U) {
		if ((event.can_id == kDmId1) || (event.can_id == kDmId2) || (event.can_id == kDmId3) ||
		    (event.can_id == 0x202U) || (event.can_id == 0x203U)) {
			(void)rm_test::app::channels::can_raw_frame_queue::EnqueueForArm(&frame);
		}
		if (event.can_id == 0x201U) {
			(void)rm_test::app::channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
	}
}

void CanRxCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);

	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		return;
	}

	CanRxEvent event = {};
	event.bus = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(user_data));
	event.can_id = static_cast<uint16_t>(frame->id);
	event.dlc = frame->dlc;

	const uint8_t copy_len = (frame->dlc > 8U) ? 8U : frame->dlc;
	for (uint8_t i = 0U; i < copy_len; ++i) {
		event.data[i] = frame->data[i];
	}

	(void)k_msgq_put(&g_can_rx_msgq, &event, K_NO_WAIT);
}

void CanDispatchThreadMain()
{
	printk("can_dispatch started\n");

	while (true) {
		CanRxEvent event = {};
		if (k_msgq_get(&g_can_rx_msgq, &event, K_FOREVER) != 0) {
			continue;
		}

		RouteCanFrameToModuleQueues(event);
	}
}

int AddDefaultFilters(const struct device *dev, uint8_t bus)
{
	const struct can_filter filter = {
		.id = 0U,
		.mask = CAN_STD_ID_MASK,
		.flags = 0U,
	};

	const uint16_t filter_ids[] = {
		kDjiId1,
		kDjiId2,
		kDjiId3,
		kDjiId4,
		kDmId1,
		kDmId2,
		kDmId3,
		kCubemarsBroadcastId,
	};

	for (size_t i = 0U; i < (sizeof(filter_ids) / sizeof(filter_ids[0])); ++i) {
		struct can_filter f = filter;
		f.id = filter_ids[i];
		if (can_add_rx_filter(
			    dev,
			    CanRxCallback,
			    reinterpret_cast<void *>(static_cast<uintptr_t>(bus)),
			    &f) < 0) {
			return -EIO;
		}
	}

	return 0;
}

}  // namespace

namespace rm_test::platform::drivers::communication::can_dispatch {

int Initialize()
{
	if (g_started) {
		return 0;
	}

	bool has_any_bus = false;
	for (uint8_t bus = 1U; bus <= kBusCount; ++bus) {
		g_can_dev[bus - 1U] = FindCanDeviceForBus(bus);
		if (g_can_dev[bus - 1U] == nullptr) {
			continue;
		}

		has_any_bus = true;

		const int rc = can_start(g_can_dev[bus - 1U]);
		if ((rc != 0) && (rc != -EALREADY)) {
			return rc;
		}

		const int filter_rc = AddDefaultFilters(g_can_dev[bus - 1U], bus);
		if (filter_rc != 0) {
			return filter_rc;
		}
	}

	if (!has_any_bus) {
		return -ENODEV;
	}

	k_thread_create(&g_can_dispatch_thread,
			g_can_dispatch_stack,
			K_THREAD_STACK_SIZEOF(g_can_dispatch_stack),
			[](void *p1, void *p2, void *p3) {
				ARG_UNUSED(p1);
				ARG_UNUSED(p2);
				ARG_UNUSED(p3);
				CanDispatchThreadMain();
			},
			nullptr,
			nullptr,
			nullptr,
			K_PRIO_PREEMPT(kThreadPrio),
			0,
			K_NO_WAIT);

	k_thread_name_set(&g_can_dispatch_thread, "can_dispatch");
	g_started = true;
	return 0;
}

int SendStdData(uint16_t can_id, const uint8_t data[8], uint8_t dlc)
{
	return SendStdDataOnBus(CanBus::kCan3, can_id, data, dlc);
}

int SendStdDataOnBus(CanBus bus, uint16_t can_id, const uint8_t data[8], uint8_t dlc)
{
	if ((data == nullptr) || (dlc > 8U)) {
		return -EINVAL;
	}

	const int idx = BusToIndex(bus);
	if (idx < 0) {
		return -EINVAL;
	}

	if (g_can_dev[idx] == nullptr) {
		return -ENODEV;
	}

	struct can_frame frame = {};
	frame.flags = 0U;
	frame.id = can_id;
	frame.dlc = dlc;
	for (uint8_t i = 0U; i < dlc; ++i) {
		frame.data[i] = data[i];
	}

	return can_send(g_can_dev[idx], &frame, K_NO_WAIT, nullptr, nullptr);
}
}  // namespace rm_test::platform::drivers::communication::can_dispatch
