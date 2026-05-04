/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_CHANNELS_CHASSIS_COMMAND_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_CHASSIS_COMMAND_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

enum ChassisInputSource : uint8_t {
	kInputSourceUnknown = 0,
	kInputSourceDr16 = 1,
	kInputSourceVt03 = 2,
};

struct ChassisCommandMessage {
	float target_vx;
	float target_vy;
	float target_wz;
	ChassisInputSource source;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_chassis_command_chan);

#endif /* RM_TEST_APP_CHANNELS_CHASSIS_COMMAND_CHANNEL_H_ */
