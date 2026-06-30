/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/irq.h>

#include <protocols/motors/dji_motor_protocol.h>

namespace protocols::motors::dji {

namespace {

int WriteBe16ToSlot(int16_t value, int slot, uint8_t out[8])
{
	if ((out == nullptr) || (slot < 0) || (slot > 3)) {
		return -EINVAL;
	}

	const uint16_t encoded = static_cast<uint16_t>(value);
	out[slot * 2] = static_cast<uint8_t>((encoded >> 8) & 0xFFU);
	out[slot * 2 + 1] = static_cast<uint8_t>(encoded & 0xFFU);
	return 0;
}

constexpr uint16_t kDjiId1 = 0x201;
constexpr uint16_t kDjiId2 = 0x202;
constexpr uint16_t kDjiId3 = 0x203;
constexpr uint16_t kDjiId4 = 0x204;


int MotorIndexFromId(uint16_t can_id)
{
	switch (can_id) {
	case kDjiId1:
		return 0;
	case kDjiId2:
		return 1;
	case kDjiId3:
		return 2;
	case kDjiId4:
		return 3;
	default:
		return -1;
	}
}

int EncodeCurrentFrameCommon(const int16_t current_cmd[4], uint8_t out[8])
{
	if ((current_cmd == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	for (int i = 0; i < 4; ++i) {
		const int rc = WriteBe16ToSlot(current_cmd[i], i, out);
		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

}  // namespace

bool IsStandardFeedbackId(uint16_t can_id)
{
	return (can_id >= 0x201U) && (can_id <= 0x204U);
}

int DecodeFeedback(const uint8_t *data, uint8_t dlc, DjiMotorFeedback *out)
{
	if ((data == nullptr) || (out == nullptr) || (dlc < 7U)) {
		return -EINVAL;
	}

	out->encoder = sys_get_be16(&data[0]);
	out->omega = static_cast<int16_t>(sys_get_be16(&data[2]));
	out->current = static_cast<int16_t>(sys_get_be16(&data[4]));
	out->temperature = data[6];
	return 0;
}


int EncodeCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8])
{
	return EncodeCurrentFrameCommon(current_cmd, out);
}

int EncodeCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8])
{
	return EncodeCurrentFrameCommon(current_cmd, out);
}

int WriteCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd, uint8_t frame_payload[8])
{
	int slot = -1;
	if ((motor_can_id >= 0x201U) && (motor_can_id <= 0x204U)) {
		slot = static_cast<int>(motor_can_id - 0x201U);
	} else {
		return -EINVAL;
	}

	return WriteBe16ToSlot(current_cmd, slot, frame_payload);
}

}  // namespace protocols::motors::dji
