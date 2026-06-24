/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_
#define RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_

#include <stddef.h>
#include <stdint.h>

#include <channels/referee_state_channel.h>

namespace platform::drivers::devices::system::referee_client {

int Initialize();
int FeedBytes(const uint8_t *data, size_t len);
int GetLatestState(channels::RefereeStateMessage *out);

}  // namespace platform::drivers::devices::system::referee_client

#endif /* RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_ */
