/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_CHANNELS_MOTOR_FEEDBACK_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_MOTOR_FEEDBACK_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct MotorFeedbackMessage {
	uint8_t bus;
	uint16_t can_id;
	uint16_t encoder;
	int16_t omega;
	int16_t current;
	uint8_t temperature;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_motor_feedback_chan);

#endif /* RM_TEST_APP_CHANNELS_MOTOR_FEEDBACK_CHANNEL_H_ */
