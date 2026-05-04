/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/irq.h>

#include <app/protocols/motors/dji_motor_protocol.h>

namespace rm_test::app::protocols::motors::dji {

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

rm_test::app::channels::MotorFeedbackMessage g_latest_state[4] = {};
bool g_latest_state_valid[4] = {false, false, false, false};
uint32_t g_feedback_sequence = 0U;

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

int IngestCanFrame(uint8_t bus, uint16_t can_id, uint8_t dlc, const uint8_t *data)
{
	if (data == nullptr || dlc < 7U) {
		return -EINVAL;
	}

	const int idx = MotorIndexFromId(can_id);
	if (idx < 0) {
		return -EINVAL;
	}

	rm_test::app::channels::MotorFeedbackMessage state = {};
	DjiMotorFeedback feedback = {};
	const int decode_rc = DecodeFeedback(data, dlc, &feedback);
	if (decode_rc != 0) {
		return decode_rc;
	}

	state.bus = bus;
	state.can_id = can_id;
	state.encoder = feedback.encoder;
	state.omega = feedback.omega;
	state.current = feedback.current;
	state.temperature = feedback.temperature;
	state.sequence = ++g_feedback_sequence;

	unsigned int key = irq_lock();
	g_latest_state[idx] = state;
	g_latest_state_valid[idx] = true;
	irq_unlock(key);

	return 0;
}

int GetLatestState(uint16_t can_id, rm_test::app::channels::MotorFeedbackMessage *out)
{
	if (out == nullptr) {
		return -EINVAL;
	}

	const int idx = MotorIndexFromId(can_id);
	if (idx < 0) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();
	if (!g_latest_state_valid[idx]) {
		irq_unlock(key);
		return -ENOENT;
	}

	*out = g_latest_state[idx];
	irq_unlock(key);
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

}  // namespace rm_test::app::protocols::motors::dji
