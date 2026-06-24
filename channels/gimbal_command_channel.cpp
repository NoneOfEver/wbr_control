/* SPDX-License-Identifier: Apache-2.0 */

#include <channels/gimbal_command_channel.h>

ZBUS_CHAN_DEFINE(rm_test_gimbal_command_chan,
		 channels::GimbalCommandMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.yaw_delta_deg = 0.0f,
			       .pitch_delta_deg = 0.0f,
			       .source = channels::kInputSourceUnknown,
			       .enable = 0U,
			       .sequence = 0U));
