/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
#include <platform/drivers/communication/can_dispatch.h>
#endif
#include <app/protocols/motors/dji_motor_protocol.h>

#include <app/services/actuator/actuator_service.h>

namespace rm_test::app::services::actuator {

namespace {

using EncodeCurrentFrameFn = int (*)(const int16_t current_cmd[4], uint8_t out[8]);

struct CurrentDispatchEntry {
	MotorCurrentGroup group;
	uint16_t can_id;
	EncodeCurrentFrameFn encode;
};

constexpr CurrentDispatchEntry kDispatchTable[] = {
	{MotorCurrentGroup::kDji0x200, 0x200U, rm_test::app::protocols::motors::dji::EncodeCurrentFrame0x200},
	{MotorCurrentGroup::kDji0x1ff, 0x1FFU, rm_test::app::protocols::motors::dji::EncodeCurrentFrame0x1ff},
};

const CurrentDispatchEntry *FindDispatchEntry(MotorCurrentGroup group)
{
	for (const auto &entry : kDispatchTable) {
		if (entry.group == group) {
			return &entry;
		}
	}

	return nullptr;
}

int EncodeCurrentFrame(const CurrentDispatchEntry *entry,
			       const int16_t current_cmd[4],
			       uint8_t frame[8])
{
	if ((entry == nullptr) || (current_cmd == nullptr) || (frame == nullptr)) {
		return -EINVAL;
	}

	return entry->encode(current_cmd, frame);
}

}  // namespace

int SendMotorCurrent(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
		     MotorCurrentGroup group,
		     const int16_t current_cmd[4])
{
	if (current_cmd == nullptr) {
		return -EINVAL;
	}

	const CurrentDispatchEntry *entry = FindDispatchEntry(group);
	if (entry == nullptr) {
		return -ENOTSUP;
	}

	uint8_t frame[8] = {0U};
	const int encode_rc = EncodeCurrentFrame(entry, current_cmd, frame);
	if (encode_rc != 0) {
		return encode_rc;
	}

	#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
	return rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
		bus,
		entry->can_id,
		frame,
		8U);
	#else
	ARG_UNUSED(frame);
	ARG_UNUSED(entry);
	ARG_UNUSED(bus);
	return -ENOTSUP;
	#endif
}

int SendDjiCurrentGroup200(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
			   const int16_t current_cmd[4])
{
	return SendMotorCurrent(bus, MotorCurrentGroup::kDji0x200, current_cmd);
}

int SendDjiCurrentGroup1ff(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
			   const int16_t current_cmd[4])
{
	return SendMotorCurrent(bus, MotorCurrentGroup::kDji0x1ff, current_cmd);
}

}  // namespace rm_test::app::services::actuator
