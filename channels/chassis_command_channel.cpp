/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/channels/chassis_command_channel.h>

ZBUS_CHAN_DEFINE(rm_test_chassis_command_chan,
		 rm_test::app::channels::ChassisCommandMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.target_vx = 0.0f,
			       .target_vy = 0.0f,
			       .target_wz = 0.0f,
			       .source = rm_test::app::channels::kInputSourceUnknown,
			       .sequence = 0U));
