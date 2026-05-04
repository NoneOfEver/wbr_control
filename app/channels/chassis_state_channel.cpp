/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/channels/chassis_state_channel.h>

ZBUS_CHAN_DEFINE(rm_test_chassis_state_chan,
		 rm_test::app::channels::ChassisStateMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.wheel1_target_omega = 0.0f,
			       .wheel2_target_omega = 0.0f,
			       .wheel3_target_omega = 0.0f,
			       .wheel4_target_omega = 0.0f,
			       .sequence = 0U));
