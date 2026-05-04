/* SPDX-License-Identifier: Apache-2.0 */

#include <app/channels/referee_state_channel.h>

ZBUS_CHAN_DEFINE(rm_test_referee_state_chan,
		 rm_test::app::channels::RefereeStateMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.current_hp = 0U,
			       .max_hp = 0U,
			       .chassis_power_limit = 0U,
			       .game_type = 0U,
			       .game_progress = 0U,
			       .stage_remain_time = 0U,
			       .bullet_type = 0U,
			       .launching_frequency = 0U,
			       .initial_speed = 0.0f,
			       .gimbal_power_on = 0U,
			       .sequence = 0U));
