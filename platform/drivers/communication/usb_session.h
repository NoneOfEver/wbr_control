/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_PLATFORM_DRIVERS_COMMUNICATION_USB_SESSION_H_
#define RM_TEST_PLATFORM_DRIVERS_COMMUNICATION_USB_SESSION_H_

#include <stddef.h>
#include <stdint.h>

namespace rm_test::platform::drivers::communication::usb_session {

int Initialize();
bool IsConfigured();
int Send(const uint8_t *data, size_t len);
int Receive(uint8_t *out, size_t capacity, size_t *out_len, int32_t timeout_ms);

}  // namespace rm_test::platform::drivers::communication::usb_session

#endif /* RM_TEST_PLATFORM_DRIVERS_COMMUNICATION_USB_SESSION_H_ */
