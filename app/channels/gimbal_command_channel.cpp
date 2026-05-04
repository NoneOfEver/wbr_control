/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/gimbal_command_channel.h>

ZBUS_CHAN_DEFINE(rm_test_gimbal_command_chan,
		 rm_test::app::channels::GimbalCommandMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.yaw_delta_deg = 0.0f,
			       .pitch_delta_deg = 0.0f,
			       .source = rm_test::app::channels::kInputSourceUnknown,
			       .enable = 0U,
			       .sequence = 0U));
