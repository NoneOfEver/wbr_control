/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/gantry_command_channel.h>

ZBUS_CHAN_DEFINE(rm_test_gantry_command_chan,
		 rm_test::app::channels::GantryCommandMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.x_delta = 0.0f,
			       .y_delta = 0.0f,
			       .z_delta = 0.0f,
			       .source = rm_test::app::channels::kInputSourceUnknown,
			       .enable = 0U,
			       .sequence = 0U));
