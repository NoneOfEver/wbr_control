/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <channels/chassismotors_send_raw.hpp>

SeqlockValue<ChassisMotorSendRawFrame> left_wheel_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_wheel_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> left_B_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> left_D_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_B_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_D_motor_send_raw;