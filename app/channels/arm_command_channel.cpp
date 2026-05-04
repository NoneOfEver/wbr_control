/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/arm_command_channel.h>

ZBUS_CHAN_DEFINE(rm_test_arm_command_chan,
		 rm_test::app::channels::ArmCommandMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.claw_delta = 0.0f,
			       .elbow_pitch_delta = 0.0f,
			       .elbow_yaw_delta = 0.0f,
			       .wrist_twist_delta = 0.0f,
			       .wrist_flip_delta = 0.0f,
			       .source = rm_test::app::channels::kInputSourceUnknown,
			       .enable = 0U,
			       .sequence = 0U));
