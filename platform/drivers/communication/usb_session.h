/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace platform::drivers::communication::usb_session {

int Initialize();
bool IsConfigured();
int Send(const uint8_t *data, size_t len);
int Receive(uint8_t *out, size_t capacity, size_t *out_len, int32_t timeout_ms);

}  // namespace platform::drivers::communication::usb_session

