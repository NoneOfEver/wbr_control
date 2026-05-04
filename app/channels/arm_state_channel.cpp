/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/arm_state_channel.h>

ZBUS_CHAN_DEFINE(rm_test_arm_state_chan,
		 rm_test::app::channels::ArmStateMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.claw_virtual_angle = 0.0f,
			       .elbow_pitch_virtual_angle = 0.0f,
			       .elbow_yaw_virtual_angle = 0.0f,
			       .wrist_left_virtual_angle = 0.0f,
			       .wrist_right_virtual_angle = 0.0f,
			       .sequence = 0U));
