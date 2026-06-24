/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_ARM_COMMAND_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_ARM_COMMAND_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

#include <channels/chassis_command_channel.h>

namespace channels {

struct ArmCommandMessage {
	float claw_delta;
	float elbow_pitch_delta;
	float elbow_yaw_delta;
	float wrist_twist_delta;
	float wrist_flip_delta;
	ChassisInputSource source;
	uint8_t enable;
	uint32_t sequence;
};

}  // namespace channels

ZBUS_CHAN_DECLARE(rm_test_arm_command_chan);

#endif /* RM_TEST_APP_CHANNELS_ARM_COMMAND_CHANNEL_H_ */
