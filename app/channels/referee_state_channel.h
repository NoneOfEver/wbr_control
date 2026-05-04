/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_REFEREE_STATE_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_REFEREE_STATE_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct RefereeStateMessage {
	uint16_t current_hp;
	uint16_t max_hp;
	uint16_t chassis_power_limit;
	uint8_t game_type;
	uint8_t game_progress;
	uint16_t stage_remain_time;
	uint8_t bullet_type;
	uint8_t launching_frequency;
	float initial_speed;
	uint8_t gimbal_power_on;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_referee_state_chan);

#endif /* RM_TEST_APP_CHANNELS_REFEREE_STATE_CHANNEL_H_ */
