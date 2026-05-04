/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_PROTOCOLS_MOTORS_DJI_MOTOR_PROTOCOL_H_
#define RM_TEST_APP_PROTOCOLS_MOTORS_DJI_MOTOR_PROTOCOL_H_

#include <stdint.h>

#include <app/channels/motor_feedback_channel.h>

namespace rm_test::app::protocols::motors::dji {

struct DjiMotorFeedback {
	uint16_t encoder;
	int16_t omega;
	int16_t current;
	uint8_t temperature;
};

bool IsStandardFeedbackId(uint16_t can_id);
int DecodeFeedback(const uint8_t *data, uint8_t dlc, DjiMotorFeedback *out);
int IngestCanFrame(uint8_t bus, uint16_t can_id, uint8_t dlc, const uint8_t *data);
int GetLatestState(uint16_t can_id, rm_test::app::channels::MotorFeedbackMessage *out);

// Pack 4 motor currents for CAN ID 0x200 (0x201~0x204).
int EncodeCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8]);
// Pack 4 motor currents for CAN ID 0x1FF.
int EncodeCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8]);

// Write one motor current command into a 0x200/0x1FF payload slot.
int WriteCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd, uint8_t frame_payload[8]);

}  // namespace rm_test::app::protocols::motors::dji

#endif /* RM_TEST_APP_PROTOCOLS_MOTORS_DJI_MOTOR_PROTOCOL_H_ */
