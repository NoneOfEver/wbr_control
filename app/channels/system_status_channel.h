/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_CHANNELS_SYSTEM_STATUS_CHANNEL_H_
#define RM_TEST_APP_CHANNELS_SYSTEM_STATUS_CHANNEL_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace rm_test::app::channels {

enum BootPhase : uint8_t {
	kBooting = 0,
	kModulesInitialized = 1,
	kRunning = 2,
};

struct SystemStatusMessage {
	BootPhase phase;
	uint32_t active_modules;
};

}  // namespace rm_test::app::channels

ZBUS_CHAN_DECLARE(rm_test_system_status_chan);

#endif /* RM_TEST_APP_CHANNELS_SYSTEM_STATUS_CHANNEL_H_ */
