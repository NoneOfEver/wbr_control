/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>

#include <algorithm>
#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/chassismotors_send_raw.hpp>
#include <modules/thread_utils.h>
#include <modules/chassis/chassis_module.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>
namespace {

K_THREAD_STACK_DEFINE(g_chassis_module_stack, 4096);
constexpr uint8_t kLeftLegBus = 0U;
constexpr uint8_t kRightLegBus = 1U;
constexpr uint16_t kLeftJointBCanId = 0x00U;
constexpr uint16_t kLeftJointBMasterId = 0x10U;
constexpr uint16_t kLeftJointDCanId = 0x03U;
constexpr uint16_t kLeftJointDMasterId = 0x13U;
constexpr uint16_t kRightJointBCanId = 0x01U;
constexpr uint16_t kRightJointBMasterId = 0x11U;
constexpr uint16_t kRightJointDCanId = 0x02U;
constexpr uint16_t kRightJointDMasterId = 0x12U;

constexpr double kDefaultTargetLegLength = 0.18;
constexpr double kDefaultTargetLegAngle = 0.0;
constexpr double kDefaultControlDt = 0.001;
constexpr uint32_t kLegFeedbackTracePeriod = 1000U;
constexpr uint32_t kLegTorqueTracePeriod = 1000U;
constexpr int kLeftLegKinematicBranch = 1;
constexpr int kRightLegKinematicBranch = -1;

constexpr protocols::motors::dm::DmMitRange kDmJointMitRange = {
	.p_min = -12.56637f,
	.p_max = 12.56637f,
	.v_min = -45.0f,
	.v_max = 45.0f,
	.kp_min = 0.0f,
	.kp_max = 500.0f,
	.kd_min = 0.0f,
	.kd_max = 5.0f,
	.t_min = -54.0f,
	.t_max = 54.0f,
};

float UIntToFloat(uint16_t value, float min_value, float max_value, uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = static_cast<float>((1U << bits) - 1U);
	return static_cast<float>(value) * span / max_int + min_value;
}

double DmPositionRad(const protocols::motors::dm::DmMotorFeedbackNormal &feedback)
{
	return UIntToFloat(feedback.angle, kDmJointMitRange.p_min, kDmJointMitRange.p_max, 16);
}

double DmVelocityRadPerSec(const protocols::motors::dm::DmMotorFeedbackNormal &feedback)
{
	return UIntToFloat(feedback.omega, kDmJointMitRange.v_min, kDmJointMitRange.v_max, 12);
}

void PublishRawCanFrame(SeqlockValue<ChassisMotorSendRawFrame> &slot,
			uint8_t bus,
			uint16_t can_id,
			const uint8_t data[8])
{
	ChassisMotorSendRawFrame frame = {};
	frame.bus = bus;
	frame.can_id = can_id;
	frame.dlc = 8U;
	for (size_t i = 0U; i < sizeof(frame.data); ++i) {
		frame.data[i] = data[i];
	}
	slot.write(frame);
}



}  // namespace

namespace modules::chassis {

int ChassisModule::Initialize()
{
	started_ = false;
	loop_ticks_ = 0U;
	left_wheel_state_ = {};
	right_wheel_state_ = {};
	left_joint_B_state = {};
	left_joint_D_state = {};
	right_joint_B_state = {};
	right_joint_D_state = {};
	leg_length_controller_.Reset();
	leg_length_controller_.SetLqrEnabled(false);
	leg_length_controller_.SetYawEnabled(false);
	leg_length_controller_.InitializeLegTarget(kDefaultTargetLegLength, kDefaultTargetLegAngle);

	return 0;
}


int ChassisModule::Start()
{
	if (started_) {
		return 0;
	}

	::modules::StartMemberThread<ChassisModule, &ChassisModule::RunLoop>(
		&thread_,
		g_chassis_module_stack,
		K_THREAD_STACK_SIZEOF(g_chassis_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"chassis_module");
	started_ = true;
	return 0;
}

void ChassisModule::RunLoop()
{
	printk("chassis module started\n");

	ChassisMotorFeedbackRawFrame left_wheel_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_wheel_feedback_local = {};
	ChassisMotorFeedbackRawFrame left_B_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame left_D_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_B_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_D_motor_feedback_local = {};

	for (;;) {

		const uint32_t left_B_feedback_sequence = left_B_motor_feedback_raw.sequence();
		const uint32_t left_D_feedback_sequence = left_D_motor_feedback_raw.sequence();
		const uint32_t right_B_feedback_sequence = right_B_motor_feedback_raw.sequence();
		const uint32_t right_D_feedback_sequence = right_D_motor_feedback_raw.sequence();

		left_wheel_feedback_raw.read(left_wheel_feedback_local);
		right_wheel_feedback_raw.read(right_wheel_feedback_local);
		left_B_motor_feedback_raw.read(left_B_motor_feedback_local);
		left_D_motor_feedback_raw.read(left_D_motor_feedback_local);
		right_B_motor_feedback_raw.read(right_B_motor_feedback_local);
		right_D_motor_feedback_raw.read(right_D_motor_feedback_local);

		(void)protocols::motors::dji::DecodeFeedback(left_wheel_feedback_local.data, 8, &left_wheel_state_);
		(void)protocols::motors::dji::DecodeFeedback(right_wheel_feedback_local.data, 8, &right_wheel_state_);
		(void)protocols::motors::dm::DecodeFeedbackNormal(left_B_motor_feedback_local.data, 8, &left_joint_B_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(left_D_motor_feedback_local.data, 8, &left_joint_D_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(right_B_motor_feedback_local.data, 8, &right_joint_B_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(right_D_motor_feedback_local.data, 8, &right_joint_D_state);

		wbr::v2::GroundBalanceInput controller_input = {};
		controller_input.control_dt = kDefaultControlDt;
		controller_input.target_leg_length = kDefaultTargetLegLength;
		controller_input.target_leg_angle = kDefaultTargetLegAngle;
		controller_input.total_mass = 8.18;
		controller_input.gravity_magnitude = 9.80665;
		if ((left_B_feedback_sequence != 0U) && (left_D_feedback_sequence != 0U)) {
			controller_input.leg_valid[0] = wbr::v2::ComputeLegKinematics(
				DmPositionRad(left_joint_D_state),
				DmPositionRad(left_joint_B_state),
				DmVelocityRadPerSec(left_joint_D_state),
				DmVelocityRadPerSec(left_joint_B_state),
				kLeftLegKinematicBranch,
				controller_input.leg[0]);
		}
		if ((right_B_feedback_sequence != 0U) && (right_D_feedback_sequence != 0U)) {
			controller_input.leg_valid[1] = wbr::v2::ComputeLegKinematics(
				DmPositionRad(right_joint_D_state),
				DmPositionRad(right_joint_B_state),
				DmVelocityRadPerSec(right_joint_D_state),
				DmVelocityRadPerSec(right_joint_B_state),
				kRightLegKinematicBranch,
				controller_input.leg[1]);
		}

		if ((loop_ticks_ % kLegFeedbackTracePeriod) == 0U) {
			printk("[chassis] leg feedback seq LB=%u LD=%u RB=%u RD=%u valid L=%u R=%u\n",
			       static_cast<unsigned int>(left_B_feedback_sequence),
			       static_cast<unsigned int>(left_D_feedback_sequence),
			       static_cast<unsigned int>(right_B_feedback_sequence),
			       static_cast<unsigned int>(right_D_feedback_sequence),
			       controller_input.leg_valid[0] ? 1U : 0U,
			       controller_input.leg_valid[1] ? 1U : 0U);
		}

		const wbr::v2::GroundBalanceOutput controller_output =
			leg_length_controller_.Update(controller_input);
		const auto motor_index = [](wbr::control::MotorId id) {
			return static_cast<int>(id);
		};

		if (loop_ticks_ < kMitEnterRepeatTicks) {
			SendDmEnterFrames();
		} else {
			const auto &left_b = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kLeftJointB)];
			const auto &left_d = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kLeftJointD)];
			const auto &right_b = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kRightJointB)];
			const auto &right_d = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kRightJointD)];
			if ((loop_ticks_ % kLegTorqueTracePeriod) == 0U) {
				printk("[chassis] leg len L=%d R=%d torque LB=%d LD=%d RB=%d RD=%d en=%u%u%u%u\n",
				       static_cast<int>(controller_input.leg[0].length * 1000.0),
				       static_cast<int>(controller_input.leg[1].length * 1000.0),
				       static_cast<int>(left_b.torque * 1000.0),
				       static_cast<int>(left_d.torque * 1000.0),
				       static_cast<int>(right_b.torque * 1000.0),
				       static_cast<int>(right_d.torque * 1000.0),
				       left_b.enabled ? 1U : 0U,
				       left_d.enabled ? 1U : 0U,
				       right_b.enabled ? 1U : 0U,
				       right_d.enabled ? 1U : 0U);
			}
			SendDmTorqueCommand(kLeftLegBus, kLeftJointBCanId,
					    left_b.enabled ? left_b.torque : 0.0);
			SendDmTorqueCommand(kLeftLegBus, kLeftJointDCanId,
					    left_d.enabled ? left_d.torque : 0.0);
			SendDmTorqueCommand(kRightLegBus, kRightJointBCanId,
					    right_b.enabled ? right_b.torque : 0.0);
			SendDmTorqueCommand(kRightLegBus, kRightJointDCanId,
					    right_d.enabled ? right_d.torque : 0.0);
		}
		++loop_ticks_;

		k_sleep(K_MSEC(1));
	}
}

void ChassisModule::SendDmEnterFrames()
{
	uint8_t data[8] = {};
	if (protocols::motors::dm::GetControlCommandFrame(
		    protocols::motors::dm::DmControlCommand::kEnter, data) != 0) {
		return;
	}
	PublishRawCanFrame(left_B_motor_send_raw, kLeftLegBus, kLeftJointBCanId, data);
	PublishRawCanFrame(left_D_motor_send_raw, kLeftLegBus, kLeftJointDCanId, data);
	PublishRawCanFrame(right_B_motor_send_raw, kRightLegBus, kRightJointBCanId, data);
	PublishRawCanFrame(right_D_motor_send_raw, kRightLegBus, kRightJointDCanId, data);
}

void ChassisModule::SendDmTorqueCommand(uint8_t bus, uint16_t can_id, double torque)
{
	protocols::motors::dm::DmMitCommand command = {};
	command.position = 0.0f;
	command.velocity = 0.0f;
	command.kp = 0.0f;
	command.kd = 0.0f;
	command.torque = static_cast<float>(torque);

	uint8_t data[8] = {};
	if (protocols::motors::dm::PackMitCommand(&command, &kDmJointMitRange, data) != 0) {
		return;
	}

	if ((bus == kLeftLegBus) && (can_id == kLeftJointBCanId)) {
		PublishRawCanFrame(left_B_motor_send_raw, bus, can_id, data);
	} else if ((bus == kLeftLegBus) && (can_id == kLeftJointDCanId)) {
		PublishRawCanFrame(left_D_motor_send_raw, bus, can_id, data);
	} else if ((bus == kRightLegBus) && (can_id == kRightJointBCanId)) {
		PublishRawCanFrame(right_B_motor_send_raw, bus, can_id, data);
	} else if ((bus == kRightLegBus) && (can_id == kRightJointDCanId)) {
		PublishRawCanFrame(right_D_motor_send_raw, bus, can_id, data);
	}
}



}  // namespace modules::chassis
