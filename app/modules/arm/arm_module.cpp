/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <algorithm>
#include <math.h>

#include <zephyr/sys/printk.h>

#include <app/bootstrap/thread_utils.h>
#include <app/modules/arm/arm_module.h>
#include <app/protocols/motors/dm_motor_protocol.h>
#include <app/protocols/motors/dji_motor_protocol.h>
#include <app/services/actuator/actuator_service.h>
#include <platform/drivers/communication/can_dispatch.h>

namespace {

K_THREAD_STACK_DEFINE(g_arm_module_stack, 1024);
constexpr bool kBaselineTraceEnabled = false;
constexpr uint32_t kBaselineTracePeriod = 250U;
constexpr uint16_t kDmClawCanId = 0x011;
constexpr uint16_t kDmElbowYawCanId = 0x012;
constexpr uint16_t kDmElbowPitchCanId = 0x013;
constexpr uint16_t kWristMotorCanId[2] = {0x202, 0x203};

int SendDmFrameOnCan3(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && (CONFIG_RM_TEST_RUNTIME_INIT_CAN == 1)
	return rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
		rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan3,
		can_id,
		data,
		dlc);
#else
	ARG_UNUSED(can_id);
	ARG_UNUSED(data);
	ARG_UNUSED(dlc);
	return -ENOTSUP;
#endif
}

int SendDmMitCommand(uint16_t can_id, float position, float kp, float kd)
{
	rm_test::app::protocols::motors::dm::DmMitRange range = {
		.p_min = -12.5f,
		.p_max = 12.5f,
		.v_min = -10.0f,
		.v_max = 10.0f,
		.kp_min = 0.0f,
		.kp_max = 500.0f,
		.kd_min = 0.0f,
		.kd_max = 5.0f,
		.t_min = -29.0f,
		.t_max = 29.0f,
	};

	rm_test::app::protocols::motors::dm::DmMitCommand cmd = {
		.position = position,
		.velocity = 0.0f,
		.kp = kp,
		.kd = kd,
		.torque = 0.0f,
	};

	uint8_t frame[8] = {0U};
	const int pack_rc = rm_test::app::protocols::motors::dm::PackMitCommand(&cmd, &range, frame);
	if (pack_rc != 0) {
		return pack_rc;
	}

	return SendDmFrameOnCan3(can_id, frame, 8U);
}

}  // namespace

namespace rm_test::app::modules::arm {

int ArmModule::Initialize()
{
	started_ = false;
	dm_started_ = false;
	(void)k_mutex_init(&pid_mutex_);
	claws_virtual_angle_ = 0.0f;
	wrist_joint_left_virtual_angle_ = 0.0f;
	wrist_joint_right_virtual_angle_ = 0.0f;
	elbow_pitch_joint_virtual_angle_ = 0.0f;
	elbow_yaw_joint_virtual_angle_ = 0.0f;
	for (uint8_t i = 0U; i < kWristCount; ++i) {
		wrist_target_omega_[i] = 0.0f;
		wrist_feedback_valid_[i] = false;
		wrist_speed_pid_[i].Init(kWristPidKp,
					kWristPidKi,
					kPidKd,
					0.0f,
					kIntegralLimit,
					kCurrentLimit,
					kLoopDtSec);
		wrist_speed_pid_[i].SetIntegralError(0.0f);
	}
	dm_feedback_claw_ = {};
	dm_feedback_elbow_pitch_ = {};
	dm_feedback_elbow_yaw_ = {};
	dm_feedback_claw_valid_ = false;
	dm_feedback_elbow_pitch_valid_ = false;
	dm_feedback_elbow_yaw_valid_ = false;
	state_sequence_ = 0U;
	return 0;
}

int ArmModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<ArmModule, &ArmModule::RunLoop>(&thread_,
							     g_arm_module_stack,
							     K_THREAD_STACK_SIZEOF(g_arm_module_stack),
							     this,
							     K_PRIO_PREEMPT(8),
							     "arm_module");
	started_ = true;
	return 0;
}

void ArmModule::ControlClaw(float virtual_angle)
{
	float angle = std::clamp(virtual_angle, -kClawLimit, kClawLimit);

	if ((angle < claws_virtual_angle_) && dm_feedback_claw_valid_) {
		const float now_torque_a = static_cast<float>(dm_feedback_claw_.current_ma) / 1000.0f;
		if (fabsf(now_torque_a) >= kClawHoldCurrentLimitA) {
			return;
		}
	}

	claws_virtual_angle_ = angle;
}

void ArmModule::ControlElbowPitchJoint(float virtual_angle)
{
	elbow_pitch_joint_virtual_angle_ =
		std::clamp(virtual_angle, -kElbowPitchLimit, kElbowPitchLimit);
}

void ArmModule::ControlElbowYawJoint(float virtual_angle)
{
	elbow_yaw_joint_virtual_angle_ = std::clamp(virtual_angle, -kElbowYawLimit, kElbowYawLimit);
}

void ArmModule::ControlWristByTwistFlip(float twist_delta, float flip_delta)
{
	wrist_joint_left_virtual_angle_ += flip_delta + twist_delta;
	wrist_joint_right_virtual_angle_ += flip_delta - twist_delta;
	wrist_joint_left_virtual_angle_ = std::clamp(wrist_joint_left_virtual_angle_, -kWristLimit, kWristLimit);
	wrist_joint_right_virtual_angle_ = std::clamp(wrist_joint_right_virtual_angle_, -kWristLimit, kWristLimit);
}

void ArmModule::HandleCommand(const channels::ArmCommandMessage &command)
{
	const float inv_dt = 1.0f / kLoopDtSec;

	if (command.enable == 0U) {
		(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
		for (uint8_t i = 0U; i < kWristCount; ++i) {
			wrist_target_omega_[i] = 0.0f;
			wrist_speed_pid_[i].reset();
		}
		k_mutex_unlock(&pid_mutex_);
		return;
	}

	const float prev_left = wrist_joint_left_virtual_angle_;
	const float prev_right = wrist_joint_right_virtual_angle_;

	ControlClaw(claws_virtual_angle_ + command.claw_delta);
	ControlElbowYawJoint(elbow_yaw_joint_virtual_angle_ + command.elbow_yaw_delta);
	ControlElbowPitchJoint(elbow_pitch_joint_virtual_angle_ + command.elbow_pitch_delta);
	ControlWristByTwistFlip(command.wrist_twist_delta, command.wrist_flip_delta);

	const float wrist_left_delta = wrist_joint_left_virtual_angle_ - prev_left;
	const float wrist_right_delta = wrist_joint_right_virtual_angle_ - prev_right;
	wrist_target_omega_[0] = wrist_left_delta * inv_dt;
	wrist_target_omega_[1] = wrist_right_delta * inv_dt;
}

void ArmModule::SendDmStartupSequence()
{
	using rm_test::app::protocols::motors::dm::DmControlCommand;

	uint8_t cmd_frame[8] = {0U};
	if (rm_test::app::protocols::motors::dm::GetControlCommandFrame(
		    DmControlCommand::kClearError,
		    cmd_frame) == 0) {
		(void)SendDmFrameOnCan3(kDmClawCanId, cmd_frame, 8U);
		(void)SendDmFrameOnCan3(kDmElbowPitchCanId, cmd_frame, 8U);
		(void)SendDmFrameOnCan3(kDmElbowYawCanId, cmd_frame, 8U);
		k_sleep(K_MSEC(100));
	}

	if (rm_test::app::protocols::motors::dm::GetControlCommandFrame(
		    DmControlCommand::kEnter,
		    cmd_frame) == 0) {
		(void)SendDmFrameOnCan3(kDmClawCanId, cmd_frame, 8U);
		(void)SendDmFrameOnCan3(kDmElbowPitchCanId, cmd_frame, 8U);
		(void)SendDmFrameOnCan3(kDmElbowYawCanId, cmd_frame, 8U);
		k_sleep(K_MSEC(1000));
	}
}

void ArmModule::DecodeCanFramesInQueue()
{
	while (true) {
		rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage frame = {};
		if (rm_test::app::channels::can_raw_frame_queue::DequeueForArm(&frame) != 0) {
			break;
		}

		if (frame.bus != 3U) {
			continue;
		}

		if ((frame.can_id == kWristMotorCanId[0]) || (frame.can_id == kWristMotorCanId[1])) {
			rm_test::app::protocols::motors::dji::DjiMotorFeedback decoded = {};
			if (rm_test::app::protocols::motors::dji::DecodeFeedback(frame.data, frame.dlc, &decoded) != 0) {
				continue;
			}

			const int idx = static_cast<int>(frame.can_id - kWristMotorCanId[0]);
			if ((idx < 0) || (idx >= static_cast<int>(kWristCount))) {
				continue;
			}

			wrist_feedback_[idx].bus = frame.bus;
			wrist_feedback_[idx].can_id = frame.can_id;
			wrist_feedback_[idx].encoder = decoded.encoder;
			wrist_feedback_[idx].omega = decoded.omega;
			wrist_feedback_[idx].current = decoded.current;
			wrist_feedback_[idx].temperature = decoded.temperature;
			wrist_feedback_valid_[idx] = true;
			continue;
		}

		if ((frame.can_id == kDmClawCanId) || (frame.can_id == kDmElbowYawCanId) ||
		    (frame.can_id == kDmElbowPitchCanId)) {
			rm_test::app::protocols::motors::dm::DmMotorFeedback1To4 decoded = {};
			if (rm_test::app::protocols::motors::dm::DecodeFeedback1To4(
				frame.data,
				frame.dlc,
				&decoded) != 0) {
				continue;
			}

			if (frame.can_id == kDmClawCanId) {
				dm_feedback_claw_ = decoded;
				dm_feedback_claw_valid_ = true;
			} else if (frame.can_id == kDmElbowPitchCanId) {
				dm_feedback_elbow_pitch_ = decoded;
				dm_feedback_elbow_pitch_valid_ = true;
			} else if (frame.can_id == kDmElbowYawCanId) {
				dm_feedback_elbow_yaw_ = decoded;
				dm_feedback_elbow_yaw_valid_ = true;
			}
		}
	}
}

void ArmModule::ApplyWristSpeedPidAndSend()
{
	int16_t current_cmd[4] = {0, 0, 0, 0};

	(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
	for (uint8_t i = 0U; i < kWristCount; ++i) {
		if (!wrist_feedback_valid_[i]) {
			continue;
		}

		const float measured = static_cast<float>(wrist_feedback_[i].omega);
		const float output = wrist_speed_pid_[i].update(wrist_target_omega_[i], measured);
		const int slot = static_cast<int>(kWristMotorCanId[i] - 0x201U);
		if ((slot >= 0) && (slot < 4)) {
			current_cmd[slot] = static_cast<int16_t>(output);
		}
	}
	k_mutex_unlock(&pid_mutex_);

	(void)rm_test::app::services::actuator::SendMotorCurrent(
		rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan3,
		rm_test::app::services::actuator::MotorCurrentGroup::kDji0x200,
		current_cmd);
}

void ArmModule::RunLoop()
{
	printk("arm module started\n");
	if (!dm_started_) {
		SendDmStartupSequence();
		dm_started_ = true;
	}

	channels::ArmCommandMessage command = {};
	channels::ArmStateMessage state = {};
	while (true) {
		DecodeCanFramesInQueue();

		if (zbus_chan_read(&rm_test_arm_command_chan, &command, K_NO_WAIT) == 0) {
			HandleCommand(command);
		} else {
			(void)k_mutex_lock(&pid_mutex_, K_FOREVER);
			for (uint8_t i = 0U; i < kWristCount; ++i) {
				wrist_target_omega_[i] = 0.0f;
				wrist_speed_pid_[i].reset();
			}
			k_mutex_unlock(&pid_mutex_);
		}

		(void)SendDmMitCommand(
			kDmClawCanId,
			claws_virtual_angle_,
			kDmClawKp,
			kDmClawKd);
		(void)SendDmMitCommand(
			kDmElbowPitchCanId,
			elbow_pitch_joint_virtual_angle_,
			kDmElbowPitchKp,
			kDmElbowPitchKd);
		(void)SendDmMitCommand(
			kDmElbowYawCanId,
			elbow_yaw_joint_virtual_angle_,
			kDmElbowYawKp,
			kDmElbowYawKd);
		ApplyWristSpeedPidAndSend();

		state.claw_virtual_angle = claws_virtual_angle_;
		state.elbow_pitch_virtual_angle = elbow_pitch_joint_virtual_angle_;
		state.elbow_yaw_virtual_angle = elbow_yaw_joint_virtual_angle_;
		state.wrist_left_virtual_angle = wrist_joint_left_virtual_angle_;
		state.wrist_right_virtual_angle = wrist_joint_right_virtual_angle_;
		state.sequence = ++state_sequence_;
		(void)zbus_chan_pub(&rm_test_arm_state_chan, &state, K_NO_WAIT);

		if (kBaselineTraceEnabled && ((state_sequence_ % kBaselineTracePeriod) == 0U)) {
			printk("[baseline][arm] claw=%.3f ep=%.3f ey=%.3f wl_t=%.1f wr_t=%.1f wl_m=%d wr_m=%d c_i=%d ep_i=%d ey_i=%d\n",
			       static_cast<double>(claws_virtual_angle_),
			       static_cast<double>(elbow_pitch_joint_virtual_angle_),
			       static_cast<double>(elbow_yaw_joint_virtual_angle_),
			       static_cast<double>(wrist_target_omega_[0]),
			       static_cast<double>(wrist_target_omega_[1]),
			       static_cast<int>(wrist_feedback_[0].omega),
			       static_cast<int>(wrist_feedback_[1].omega),
			       static_cast<int>(dm_feedback_claw_.current_ma),
			       static_cast<int>(dm_feedback_elbow_pitch_.current_ma),
			       static_cast<int>(dm_feedback_elbow_yaw_.current_ma));
		}
		k_sleep(K_MSEC(2));
	}
}

}  // namespace rm_test::app::modules::arm
