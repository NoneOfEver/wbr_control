/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/channels/remote_input_channel.h>

ZBUS_CHAN_DEFINE(rm_test_remote_input_chan,
		 rm_test::app::channels::RemoteInputMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.source = rm_test::app::channels::kRemoteInputUnknown,
			       .chassis_enable = 0U,
			       .axis_lx = 0.0f,
			       .axis_ly = 0.0f,
			       .axis_wheel = 0.0f,
			       .axis_jx = 0.0f,
			       .axis_jy = 0.0f,
			       .axis_jz = 0.0f,
			       .sequence = 0U));
