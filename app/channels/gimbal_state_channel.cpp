/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/gimbal_state_channel.h>

ZBUS_CHAN_DEFINE(rm_test_gimbal_state_chan,
		 rm_test::app::channels::GimbalStateMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.yaw_angle_deg = 0.0f,
			       .pitch_angle_deg = 0.0f,
			       .servo_ready = 0U,
			       .sequence = 0U));
