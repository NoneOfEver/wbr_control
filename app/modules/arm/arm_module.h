/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_MODULES_ARM_ARM_MODULE_H_
#define RM_TEST_APP_MODULES_ARM_ARM_MODULE_H_

#include <zephyr/kernel.h>

#include <app/algorithms/control/alg_pid.h>
#include <app/bootstrap/module.h>
#include <app/channels/arm_command_channel.h>
#include <app/channels/can_raw_frame_queue.h>
#include <app/channels/motor_feedback_channel.h>
#include <app/channels/arm_state_channel.h>
#include <app/protocols/motors/dm_motor_protocol.h>

namespace rm_test::app::modules::arm {

class ArmModule : public bootstrap::Module {
public:
	const char *Name() const override { return "arm"; }
	int Initialize() override;
	int Start() override;

	void ControlClaw(float virtual_angle);
	void ControlElbowPitchJoint(float virtual_angle);
	void ControlElbowYawJoint(float virtual_angle);
	void ControlWristByTwistFlip(float twist_delta, float flip_delta);

private:
	static constexpr float kClawLimit = 0.5f;
	static constexpr float kWristLimit = 1.0f;
	static constexpr float kElbowPitchLimit = 1.0f;
	static constexpr float kElbowYawLimit = 3.14f;
	static constexpr float kLoopDtSec = 0.002f;
	static constexpr float kWristPidKp = 1.0f;
	static constexpr float kWristPidKi = 0.05f;
	static constexpr float kPidKd = 0.0f;
	static constexpr float kIntegralLimit = 500.0f;
	static constexpr float kCurrentLimit = 16384.0f;
	static constexpr uint8_t kWristCount = 2U;
	static constexpr float kClawHoldCurrentLimitA = 2.0f;
	static constexpr float kDmPosMin = -12.5f;
	static constexpr float kDmPosMax = 12.5f;
	static constexpr float kDmVelMin = -10.0f;
	static constexpr float kDmVelMax = 10.0f;
	static constexpr float kDmKpMin = 0.0f;
	static constexpr float kDmKpMax = 500.0f;
	static constexpr float kDmKdMin = 0.0f;
	static constexpr float kDmKdMax = 5.0f;
	static constexpr float kDmTorqueMin = -29.0f;
	static constexpr float kDmTorqueMax = 29.0f;
	static constexpr float kDmClawKp = 10.0f;
	static constexpr float kDmClawKd = 0.0f;
	static constexpr float kDmElbowPitchKp = 10.0f;
	static constexpr float kDmElbowPitchKd = 0.0f;
	static constexpr float kDmElbowYawKp = 10.0f;
	static constexpr float kDmElbowYawKd = 0.0f;

	void RunLoop();
	void HandleCommand(const channels::ArmCommandMessage &command);
	void SendDmStartupSequence();
	void DecodeCanFramesInQueue();
	void ApplyWristSpeedPidAndSend();

	struct k_thread thread_;
	bool started_ = false;
	float claws_virtual_angle_ = 0.0f;
	float wrist_joint_left_virtual_angle_ = 0.0f;
	float wrist_joint_right_virtual_angle_ = 0.0f;
	float elbow_pitch_joint_virtual_angle_ = 0.0f;
	float elbow_yaw_joint_virtual_angle_ = 0.0f;
	float wrist_target_omega_[kWristCount] = {0.0f, 0.0f};
	channels::MotorFeedbackMessage wrist_feedback_[kWristCount] = {};
	bool wrist_feedback_valid_[kWristCount] = {false, false};
	rm_test::app::protocols::motors::dm::DmMotorFeedback1To4 dm_feedback_claw_ = {};
	rm_test::app::protocols::motors::dm::DmMotorFeedback1To4 dm_feedback_elbow_pitch_ = {};
	rm_test::app::protocols::motors::dm::DmMotorFeedback1To4 dm_feedback_elbow_yaw_ = {};
	bool dm_feedback_claw_valid_ = false;
	bool dm_feedback_elbow_pitch_valid_ = false;
	bool dm_feedback_elbow_yaw_valid_ = false;
	alg::Pid wrist_speed_pid_[kWristCount] = {};
	struct k_mutex pid_mutex_;
	bool dm_started_ = false;
	uint32_t state_sequence_ = 0U;
};

}  // namespace rm_test::app::modules::arm

#endif /* RM_TEST_APP_MODULES_ARM_ARM_MODULE_H_ */
