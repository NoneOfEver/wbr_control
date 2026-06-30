/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

struct SerialServoSendRawFrame {
	uint8_t valid;
	uint8_t len;
	uint8_t data[16];
};

extern SeqlockValue<SerialServoSendRawFrame> yaw_servo_send_raw;
extern SeqlockValue<SerialServoSendRawFrame> pitch_servo_send_raw;
