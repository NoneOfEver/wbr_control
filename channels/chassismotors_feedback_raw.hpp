/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

struct ChassisMotorFeedbackRawFrame {
	uint8_t data[8];
};

extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_B_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_D_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_B_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_D_motor_feedback_raw;
