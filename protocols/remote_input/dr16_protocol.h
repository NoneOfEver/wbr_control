/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_
#define RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace rm_test::app::protocols::remote_input::dr16 {

constexpr size_t kFrameLength = 18;

struct Dr16Frame {
	float right_stick_x;
	float right_stick_y;
	float left_stick_x;
	float left_stick_y;
	float wheel;
	uint8_t left_switch;
	uint8_t right_switch;
	bool chassis_enable;
};

bool DecodeFrame(const uint8_t *data, size_t len, Dr16Frame *out);

}  // namespace rm_test::app::protocols::remote_input::dr16

#endif /* RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_ */
