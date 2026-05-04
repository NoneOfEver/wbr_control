/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#include <algorithm>

#include <app/bootstrap/thread_utils.h>
#include <app/modules/chassis/chassis_module.h>
#include <app/protocols/motors/dji_motor_protocol.h>
#include <app/services/actuator/actuator_service.h>
#include <app/services/chassis/chassis_tuning_service.h>

namespace {

K_THREAD_STACK_DEFINE(g_chassis_module_stack, 1024);
constexpr bool kBaselineTraceEnabled = false;
constexpr uint32_t kBaselineTracePeriod = 500U;

}  // namespace

namespace rm_test::app::modules::chassis {

int ChassisModule::Initialize()
{
	state_ = {};
	publish_sequence_ = 0U;
	idle_ticks_ = 0U;
	started_ = false;
	(void)k_mutex_init(&pid_mutex_);

	const int unregister_rc = rm_test::app::services::chassis_tuning::UnregisterProvider(this);
	if ((unregister_rc != 0) && (unregister_rc != -ENOENT)) {
		return unregister_rc;
	}

	const int register_rc =
		rm_test::app::services::chassis_tuning::RegisterProvider(this, "chassis_module", 100);
	if (register_rc != 0) {
		return register_rc;
	}

	return SetSpeedPidTuning(kPidKp, kPidKi, kPidKd, kIntegralLimit, kCurrentLimit);
}

void ChassisModule::ResetTargetsAndPid()
{
	state_.wheel1_target_omega = 0.0f;
	state_.wheel2_target_omega = 0.0f;
	state_.wheel3_target_omega = 0.0f;
	state_.wheel4_target_omega = 0.0f;
	(void)ResetSpeedPidIntegrator();
}

int ChassisModule::SetSpeedPidTuning(float kp, float ki, float kd, float i_limit, float out_limit)
{
	if ((i_limit <= 0.0f) || (out_limit <= 0.0f)) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
	pid_kp_ = kp;
	pid_ki_ = ki;
	pid_kd_ = kd;
	pid_i_limit_ = i_limit;
	pid_out_limit_ = out_limit;

	for (int i = 0; i < 4; ++i) {
		wheel_speed_pid_[i].Init(
			pid_kp_,
			pid_ki_,
			pid_kd_,
			0.0f,
			pid_i_limit_,
			pid_out_limit_,
			0.001f);
		wheel_speed_pid_[i].SetIntegralError(0.0f);
	}
	k_mutex_unlock(&pid_mutex_);
	return 0;
}

int ChassisModule::GetSpeedPidTuning(float *kp, float *ki, float *kd, float *i_limit, float *out_limit)
{
	if ((kp == nullptr) || (ki == nullptr) || (kd == nullptr) || (i_limit == nullptr) ||
	    (out_limit == nullptr)) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
	*kp = pid_kp_;
	*ki = pid_ki_;
	*kd = pid_kd_;
	*i_limit = pid_i_limit_;
	*out_limit = pid_out_limit_;
	k_mutex_unlock(&pid_mutex_);
	return 0;
}

int ChassisModule::ResetSpeedPidIntegrator()
{
	(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
	for (int i = 0; i < 4; ++i) {
		wheel_speed_pid_[i].reset();
	}
	k_mutex_unlock(&pid_mutex_);
	return 0;
}

int ChassisModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<ChassisModule, &ChassisModule::RunLoop>(
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

	channels::ChassisCommandMessage command = {};

	while (true) {
		DecodeCanFramesInQueue();

		if (zbus_chan_read(&rm_test_chassis_command_chan, &command, K_NO_WAIT) == 0) {
			idle_ticks_ = 0U;
			UpdateStateFromCommand(command);
		} else {
			++idle_ticks_;
			if (idle_ticks_ > kNoCommandStopTicks) {
				ResetTargetsAndPid();
			}
		}

		ApplyWheelSpeedPidAndSend();

		state_.sequence = ++publish_sequence_;
		(void)zbus_chan_pub(&rm_test_chassis_state_chan, &state_, K_NO_WAIT);
		k_sleep(K_MSEC(1));
	}
}

void ChassisModule::DecodeCanFramesInQueue()
{
	while (true) {
		rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage frame = {};
		if (rm_test::app::channels::can_raw_frame_queue::DequeueForChassis(&frame) != 0) {
			break;
		}

		if (frame.bus != 1U) {
			continue;
		}

		if ((frame.can_id < 0x201U) || (frame.can_id > 0x204U)) {
			continue;
		}

		rm_test::app::protocols::motors::dji::DjiMotorFeedback decoded = {};
		if (rm_test::app::protocols::motors::dji::DecodeFeedback(frame.data, frame.dlc, &decoded) != 0) {
			continue;
		}

		const int idx = static_cast<int>(frame.can_id - 0x201U);
		if ((idx < 0) || (idx >= 4)) {
			continue;
		}

		motor_feedback_[idx].bus = frame.bus;
		motor_feedback_[idx].can_id = frame.can_id;
		motor_feedback_[idx].encoder = decoded.encoder;
		motor_feedback_[idx].omega = decoded.omega;
		motor_feedback_[idx].current = decoded.current;
		motor_feedback_[idx].temperature = decoded.temperature;
		motor_feedback_valid_[idx] = true;
	}
}

void ChassisModule::UpdateStateFromCommand(const channels::ChassisCommandMessage &command)
{
	const float vx = std::clamp(command.target_vx, -kCommandVxLimit, kCommandVxLimit);
	const float vy = std::clamp(command.target_vy, -kCommandVyLimit, kCommandVyLimit);
	const float wz = std::clamp(command.target_wz, -kCommandWzLimit, kCommandWzLimit);

	state_.wheel1_target_omega = std::clamp(
		(+kKinematicsFactor * vx - kKinematicsFactor * vy) + wz,
		-kWheelTargetOmegaLimit,
		kWheelTargetOmegaLimit);
	state_.wheel2_target_omega = std::clamp(
		(-kKinematicsFactor * vx - kKinematicsFactor * vy) + wz,
		-kWheelTargetOmegaLimit,
		kWheelTargetOmegaLimit);
	state_.wheel3_target_omega = std::clamp(
		(-kKinematicsFactor * vx + kKinematicsFactor * vy) + wz,
		-kWheelTargetOmegaLimit,
		kWheelTargetOmegaLimit);
	state_.wheel4_target_omega = std::clamp(
		(+kKinematicsFactor * vx + kKinematicsFactor * vy) + wz,
		-kWheelTargetOmegaLimit,
		kWheelTargetOmegaLimit);
}

void ChassisModule::ApplyWheelSpeedPidAndSend()
{
	const float target[4] = {
		state_.wheel1_target_omega,
		state_.wheel2_target_omega,
		state_.wheel3_target_omega,
		state_.wheel4_target_omega,
	};

	int16_t current_cmd[4] = {0, 0, 0, 0};
	(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
	for (int i = 0; i < 4; ++i) {
		if (!motor_feedback_valid_[i]) {
			continue;
		}

		const float measured = static_cast<float>(motor_feedback_[i].omega);
		const float out = wheel_speed_pid_[i].update(target[i], measured);
		current_cmd[i] = static_cast<int16_t>(out);
	}
	k_mutex_unlock(&pid_mutex_);

	(void)rm_test::app::services::actuator::SendMotorCurrent(
		rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan1,
		rm_test::app::services::actuator::MotorCurrentGroup::kDji0x200,
		current_cmd);

	if (kBaselineTraceEnabled) {
		static uint32_t trace_tick = 0U;
		if ((++trace_tick % kBaselineTracePeriod) == 0U) {
			printk("[baseline][chassis] tgt=(%.2f %.2f %.2f %.2f) meas=(%d %d %d %d) cur=(%d %d %d %d) idle=%u\n",
			       static_cast<double>(target[0]),
			       static_cast<double>(target[1]),
			       static_cast<double>(target[2]),
			       static_cast<double>(target[3]),
			       static_cast<int>(motor_feedback_[0].omega),
			       static_cast<int>(motor_feedback_[1].omega),
			       static_cast<int>(motor_feedback_[2].omega),
			       static_cast<int>(motor_feedback_[3].omega),
			       static_cast<int>(current_cmd[0]),
			       static_cast<int>(current_cmd[1]),
			       static_cast<int>(current_cmd[2]),
			       static_cast<int>(current_cmd[3]),
			       static_cast<unsigned int>(idle_ticks_));
		}
	}
}

}  // namespace rm_test::app::modules::chassis
