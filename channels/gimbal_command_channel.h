/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_GIMBAL_COMMAND_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_GIMBAL_COMMAND_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

#include <app/channels/chassis_command_channel.h>

namespace rm_test::app::channels {

struct GimbalCommandMessage {
	float yaw_delta_deg;
	float pitch_delta_deg;
	ChassisInputSource source;
	uint8_t enable;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_gimbal_command_chan);

#endif /* RM_TEST_APP_CHANNELS_GIMBAL_COMMAND_CHANNEL_H_ */
