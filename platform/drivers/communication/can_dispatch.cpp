/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <channels/can_raw_frame_queue.h>

#include "can_dispatch.h"

namespace {

constexpr uint8_t kBusCount = 3U;
constexpr uint16_t kDjiId1 = 0x201;
constexpr uint16_t kDjiId2 = 0x202;
constexpr uint16_t kDjiId3 = 0x203;
constexpr uint16_t kDjiId4 = 0x204;
constexpr uint16_t kDmId1 = 0x011;
constexpr uint16_t kDmId2 = 0x012;
constexpr uint16_t kDmId3 = 0x013;
constexpr uint16_t kCubemarsBroadcastId = 0x000;

const struct device *g_can_dev[kBusCount] = {nullptr, nullptr, nullptr};
bool g_started = false;

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

void RouteCanFrameToModuleQueues(uint8_t bus, const struct can_frame *rx_frame)
{
	if (rx_frame == nullptr) {
		return;
	}

	channels::can_raw_frame_queue::CanRawFrameMessage frame = {};
	frame.bus = bus;
	frame.can_id = static_cast<uint16_t>(rx_frame->id);
	frame.dlc = rx_frame->dlc;

	const uint8_t copy_len = (rx_frame->dlc > 8U) ? 8U : rx_frame->dlc;
	for (uint8_t i = 0U; i < copy_len; ++i) {
		frame.data[i] = rx_frame->data[i];
	}

	if (bus == 1U) {
		if ((frame.can_id >= kDjiId1) && (frame.can_id <= kDjiId4)) {
			(void)channels::can_raw_frame_queue::EnqueueForChassis(&frame);
		}
		if (frame.can_id == kCubemarsBroadcastId) {
			(void)channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
		return;
	}

	if (bus == 2U) {
		if ((frame.can_id == kCubemarsBroadcastId) || (frame.can_id == 0x201U) ||
		    (frame.can_id == 0x202U)) {
			(void)channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
		return;
	}

	if (bus == 3U) {
		if ((frame.can_id == kDmId1) || (frame.can_id == kDmId2) || (frame.can_id == kDmId3) ||
		    (frame.can_id == 0x202U) || (frame.can_id == 0x203U)) {
			(void)channels::can_raw_frame_queue::EnqueueForArm(&frame);
		}
		if (frame.can_id == 0x201U) {
			(void)channels::can_raw_frame_queue::EnqueueForGantry(&frame);
		}
	}
}

void CanRxCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);

	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		return;
	}

	const uint8_t bus = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(user_data));
	RouteCanFrameToModuleQueues(bus, frame);
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

namespace platform::drivers::communication::can_dispatch {

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

	g_started = true;
	return 0;
}

}  // namespace platform::drivers::communication::can_dispatch
