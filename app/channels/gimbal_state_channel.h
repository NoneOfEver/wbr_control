/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_GIMBAL_STATE_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_GIMBAL_STATE_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct GimbalStateMessage {
	float yaw_angle_deg;
	float pitch_angle_deg;
	uint8_t servo_ready;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_gimbal_state_chan);

#endif /* RM_TEST_APP_CHANNELS_GIMBAL_STATE_CHANNEL_H_ */
