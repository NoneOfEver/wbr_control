/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/channels/motor_feedback_channel.h>

ZBUS_CHAN_DEFINE(rm_test_motor_feedback_chan,
		 rm_test::app::channels::MotorFeedbackMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.bus = 0U,
			       .can_id = 0U,
			       .encoder = 0U,
			       .omega = 0,
			       .current = 0,
			       .temperature = 0U,
			       .sequence = 0U));
