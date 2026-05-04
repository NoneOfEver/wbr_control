/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_
#define RM_TEST_APP_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_

#include <stdint.h>

namespace rm_test::app::protocols::motors::dm {

struct DmMotorFeedback1To4 {
	uint16_t encoder;
	int16_t omega_x100;
	int16_t current_ma;
	uint8_t rotor_temperature;
	uint8_t mos_temperature;
};

enum class DmControlCommand {
	kClearError,
	kEnter,
	kExit,
	kSaveZero,
};

struct DmMitCommand {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
};

struct DmMitRange {
	float p_min;
	float p_max;
	float v_min;
	float v_max;
	float kp_min;
	float kp_max;
	float kd_min;
	float kd_max;
	float t_min;
	float t_max;
};

int DecodeFeedback1To4(const uint8_t *data, uint8_t dlc, DmMotorFeedback1To4 *out);
int GetControlCommandFrame(DmControlCommand cmd, uint8_t out[8]);
int PackMitCommand(const DmMitCommand *cmd, const DmMitRange *range, uint8_t out[8]);
int Pack1To4CurrentFrame(uint16_t motor_can_id, int16_t current_ma, uint8_t frame_payload[8]);

}  // namespace rm_test::app::protocols::motors::dm

#endif /* RM_TEST_APP_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_ */
