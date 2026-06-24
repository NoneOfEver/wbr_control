/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_ARM_STATE_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_ARM_STATE_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct ArmStateMessage {
	float claw_virtual_angle;
	float elbow_pitch_virtual_angle;
	float elbow_yaw_virtual_angle;
	float wrist_left_virtual_angle;
	float wrist_right_virtual_angle;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_arm_state_chan);

#endif /* RM_TEST_APP_CHANNELS_ARM_STATE_CHANNEL_H_ */
