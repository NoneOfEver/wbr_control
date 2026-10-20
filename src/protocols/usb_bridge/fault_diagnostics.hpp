/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_protocol.hpp"
namespace wbr::bridge {
// Optional test-only Ready payload. Snapshot BEFORE the querying Hello stops
// the old session. Normal production Hello uses page zero and is unchanged.
constexpr uint8_t kFaultQueryPage = 0xf1;
constexpr uint32_t kFaultMagic = 0x31544c46; // FLT1
constexpr uint32_t kStopSubmissionError = 0x100;
enum class StopReason : uint32_t {
  Explicit = 1,
  LeaseExpired = 2,
  UsbDeconfigured = 3,
  CanSubmissionFailed = 4
};
struct FaultSnapshot {
  uint32_t accepted = 0, rejected = 0, lease_stops = 0, explicit_stops = 0;
  uint32_t last_accepted_us = 0, last_stop_submit_us = 0, stop_flags = 0;
  uint32_t prior_session = 0;
};
inline void EncodeFaultSnapshot(Packet &packet, const FaultSnapshot &value) {
  const uint32_t words[] = {kFaultMagic,
                            value.accepted,
                            value.rejected,
                            value.lease_stops,
                            value.explicit_stops,
                            value.last_accepted_us,
                            value.last_stop_submit_us,
                            value.stop_flags,
                            value.prior_session};
  for (unsigned i = 0; i < 9; ++i)
    Put32(packet.data() + 28 + 4 * i, words[i]);
}
inline bool DecodeFaultSnapshot(const Packet &packet, FaultSnapshot &value) {
  if (U32(packet.data() + 28) != kFaultMagic)
    return false;
  value = {U32(packet.data() + 32), U32(packet.data() + 36),
           U32(packet.data() + 40), U32(packet.data() + 44),
           U32(packet.data() + 48), U32(packet.data() + 52),
           U32(packet.data() + 56), U32(packet.data() + 60)};
  return true;
}
} // namespace wbr::bridge
