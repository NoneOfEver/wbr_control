/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_CHANNELS_CHASSIS_STATE_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_CHASSIS_STATE_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct ChassisStateMessage {
	float wheel1_target_omega;
	float wheel2_target_omega;
	float wheel3_target_omega;
	float wheel4_target_omega;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_chassis_state_chan);

#endif /* RM_TEST_APP_CHANNELS_CHASSIS_STATE_CHANNEL_H_ */
