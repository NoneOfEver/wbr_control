/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/channels/system_status_channel.h>

ZBUS_CHAN_DEFINE(rm_test_system_status_chan,
		 rm_test::app::channels::SystemStatusMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.phase = rm_test::app::channels::kBooting,
			       .active_modules = 0U));
