/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_CHANNELS_REMOTE_INPUT_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_REMOTE_INPUT_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

enum RemoteInputSource : uint8_t {
	kRemoteInputUnknown = 0,
	kRemoteInputDr16 = 1,
	kRemoteInputVt03 = 2,
};

struct RemoteInputMessage {
	RemoteInputSource source;
	uint8_t chassis_enable;
	float axis_lx;
	float axis_ly;
	float axis_wheel;
	float axis_jx;
	float axis_jy;
	float axis_jz;
	uint32_t sequence;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_remote_input_chan);

#endif /* RM_TEST_APP_CHANNELS_REMOTE_INPUT_CHANNEL_H_ */
