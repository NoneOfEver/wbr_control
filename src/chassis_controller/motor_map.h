/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>
#include <chassis_controller/communication/can_dispatch.h>
#include <motors_params_generated.h>

namespace platform::motor_map {

/** CAN address used by both actuator commands and feedback routing. */
struct MotorAddress {
    uint8_t bus;
    uint16_t command_id;
    uint16_t feedback_id;
    uint8_t command_slot; // DJI group-frame index; unused for DM MIT joints.
    CanTxSlot tx_slot;
};

inline constexpr MotorAddress kLeftWheel = {
    motors_params::kLeftWheelBus, motors_params::kLeftWheelCommandId,
    motors_params::kLeftWheelFeedbackId, motors_params::kLeftWheelCommandSlot, CanTxSlot::kLeftWheel};
inline constexpr MotorAddress kRightWheel = {
    motors_params::kRightWheelBus, motors_params::kRightWheelCommandId,
    motors_params::kRightWheelFeedbackId, motors_params::kRightWheelCommandSlot, CanTxSlot::kRightWheel};
inline constexpr MotorAddress kLeftJointB = {
    motors_params::kLeftJointBBus, motors_params::kLeftJointBCommandId,
    motors_params::kLeftJointBFeedbackId, 0U, CanTxSlot::kLeftJointB};
inline constexpr MotorAddress kLeftJointD = {
    motors_params::kLeftJointDBus, motors_params::kLeftJointDCommandId,
    motors_params::kLeftJointDFeedbackId, 0U, CanTxSlot::kLeftJointD};
inline constexpr MotorAddress kRightJointB = {
    motors_params::kRightJointBBus, motors_params::kRightJointBCommandId,
    motors_params::kRightJointBFeedbackId, 0U, CanTxSlot::kRightJointB};
inline constexpr MotorAddress kRightJointD = {
    motors_params::kRightJointDBus, motors_params::kRightJointDCommandId,
    motors_params::kRightJointDFeedbackId, 0U, CanTxSlot::kRightJointD};

inline constexpr MotorAddress kMotors[] = {
    kLeftWheel, kRightWheel, kLeftJointB, kLeftJointD, kRightJointB, kRightJointD};

} // namespace platform::motor_map
