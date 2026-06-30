/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_
#define RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_

#include <array>
#include <stddef.h>
#include <stdint.h>

namespace protocols::remote_input::wfly_sbus {

constexpr size_t kFrameLength = 25;
constexpr uint8_t kStartByte = 0x0f;
constexpr uint8_t kEndByte = 0x00;
constexpr size_t kChannelCount = 16;

struct WflySbusFrame {
  std::array<uint16_t, kChannelCount> channels;
  bool channel17;
  bool channel18;
  bool frame_lost;
  bool failsafe;
};

bool DecodeFrame(const uint8_t *data, size_t len, WflySbusFrame *out);

} // namespace protocols::remote_input::wfly_sbus

#endif /* RM_TEST_APP_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_ */
