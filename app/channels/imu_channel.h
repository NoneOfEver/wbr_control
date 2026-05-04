/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_IMU_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_IMU_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

struct ImuStateMessage {
	float yaw_total_angle;
	float pitch;
	float yaw_omega;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_imu_state_chan);

#endif /* RM_TEST_APP_CHANNELS_IMU_CHANNEL_H_ */
