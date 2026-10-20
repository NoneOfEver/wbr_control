/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace wbr::bridge
{
constexpr uint32_t kMagic = 0x31425257; // WRB1, protocol version independent of tests
constexpr uint8_t kVersion = 1;
constexpr size_t kInBytes = 64, kOutBytes = 128;
constexpr uint32_t kCommandTimeoutUs = 20000;
enum class Kind : uint8_t {
	Hello = 1,
	Ready = 2,
	Commands = 3,
	Can = 4,
	Uart = 5,
	Imu = 6,
	Stop = 7
};
enum class Port : uint8_t { Remote = 1, Hi91 = 2, Referee = 3 };
using Packet = std::array<uint8_t, kInBytes>;
using Commands = std::array<uint8_t, kOutBytes>;
inline uint32_t U32(const uint8_t *p)
{
	return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void Put32(uint8_t *p, uint32_t v)
{
	for (unsigned i = 0; i < 4; ++i)
		p[i] = uint8_t(v >> (i * 8));
}
inline uint16_t U16(const uint8_t *p)
{
	return uint16_t(p[0]) | uint16_t(p[1]) << 8;
}
inline void Put16(uint8_t *p, uint16_t v)
{
	p[0] = uint8_t(v);
	p[1] = uint8_t(v >> 8);
}
inline void Header(uint8_t *p, Kind kind, uint32_t seq, uint32_t session)
{
	Put32(p, kMagic);
	Put32(p + 4, seq);
	p[8] = kVersion;
	p[9] = uint8_t(kind);
	Put32(p + 12, session);
}
inline bool HasHeader(const uint8_t *p, size_t length, Kind kind)
{
	return length >= 16 && U32(p) == kMagic && p[8] == kVersion && p[9] == uint8_t(kind);
}
inline bool NewSequence(uint32_t next, uint32_t previous)
{
	return int32_t(next - previous) > 0;
}
// CAN record: bus, length, standard ID, 8 payload bytes, MCU receive time.
inline bool ValidCan(const uint8_t *record)
{
	return (record[0] == 2 || record[0] == 3) && record[1] <= 8 && U16(record + 2) <= 0x7ff;
}
inline void CanRecord(uint8_t *p, uint8_t bus, uint16_t id, const uint8_t *data, uint8_t length,
		      uint32_t timestamp = 0)
{
	p[0] = bus;
	p[1] = length;
	Put16(p + 2, id);
	std::memcpy(p + 4, data, length);
	Put32(p + 12, timestamp);
}
inline bool ValidCommands(const Commands &packet, uint32_t session)
{
	if (!HasHeader(packet.data(), packet.size(), Kind::Commands) ||
	    U32(packet.data() + 12) != session || packet[10] > 6 || packet[11] != 0)
		return false;
	uint32_t seen = 0;
	for (unsigned i = 0; i < packet[10]; ++i) {
		const auto *p = packet.data() + 32 + i * 16;
		if (!ValidCan(p) || p[1] != 8 || p[12] >= 6)
			return false;
		if (seen & (1U << p[12]))
			return false;
		seen |= 1U << p[12];
		if (p[13] || p[14] || p[15])
			return false;
	}
	for (unsigned i = 16; i < 32; ++i)
		if (packet[i])
			return false;
	for (size_t i = 32 + packet[10] * 16; i < packet.size(); ++i)
		if (packet[i])
			return false;
	return true;
}
// The gateway renews this only after a fully validated, newly sequenced packet
// is accepted. Wrap and timestamp zero are valid; invalid traffic cannot extend
// the local motor shutdown deadline.
class CommandLease
{
public:
	bool CanAccept(uint32_t sequence) const
	{
		return !has_sequence_ || NewSequence(sequence, last_sequence_);
	}
	void Renew(uint32_t sequence, uint32_t now)
	{
		last_sequence_ = sequence;
		last_command_us_ = now;
		has_sequence_ = true;
		active_ = true;
	}
	bool Expired(uint32_t now, bool configured) const
	{
		return active_ &&
		       (!configured || uint32_t(now - last_command_us_) >= kCommandTimeoutUs);
	}
	void Reset()
	{
		active_ = false;
		has_sequence_ = false;
	}

private:
	uint32_t last_sequence_ = 0, last_command_us_ = 0;
	bool active_ = false, has_sequence_ = false;
};

// Session is negotiated by Hello/Ready before any 128-byte OUT. No periodic ACK.
// IN session ID prevents old feedback surviving reconnect. Timestamps are only
// operational freshness metadata on CAN/IMU, never a profiling/timing protocol.
} // namespace wbr::bridge
