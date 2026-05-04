/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/imu_channel.h>

ZBUS_CHAN_DEFINE(rm_test_imu_state_chan,
		 rm_test::app::channels::ImuStateMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.yaw_total_angle = 0.0f,
			       .pitch = 0.0f,
			       .yaw_omega = 0.0f,
			       .sequence = 0U));
