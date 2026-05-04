/* SPDX-License-Identifier: Apache-2.0 */

#include <algorithm>
#include <cmath>

#include <zephyr/sys/printk.h>

#include <app/bootstrap/thread_utils.h>
#include <app/modules/gimbal/gimbal_module.h>
#include <platform/drivers/devices/actuators/serial_servo.h>

namespace {

K_THREAD_STACK_DEFINE(g_gimbal_module_stack, 1024);
constexpr bool kBaselineTraceEnabled = false;
constexpr uint32_t kBaselineTracePeriod = 500U;

}  // namespace

namespace rm_test::app::modules::gimbal {

int GimbalModule::Initialize()
{
	started_ = false;
	servo_ready_ =
		(rm_test::platform::drivers::devices::actuators::serial_servo::Initialize() == 0);
	servo_stopped_ = false;
	yaw_servo_online_ = false;
	pitch_servo_online_ = false;
	command_enable_ = 0U;
	idle_ticks_ = 0U;
	yaw_servo_id_ = kDefaultYawServoId;
	pitch_servo_id_ = kDefaultPitchServoId;
	yaw_angle_deg_ = 0.0f;
	pitch_angle_deg_ = 0.0f;
	yaw_last_sent_angle_deg_ = 0.0f;
	pitch_last_sent_angle_deg_ = 0.0f;
	yaw_sent_once_ = false;
	pitch_sent_once_ = false;
	next_servo_send_ms_ = 0;
	state_sequence_ = 0U;

	if (servo_ready_) {
		uint8_t found = 0U;
		if (rm_test::platform::drivers::devices::actuators::serial_servo::ReadId(
			    yaw_servo_id_, &found, 120U) == 0) {
			yaw_servo_id_ = found;
			yaw_servo_online_ = true;
		}
		if (rm_test::platform::drivers::devices::actuators::serial_servo::ReadId(
			    pitch_servo_id_, &found, 120U) == 0) {
			pitch_servo_id_ = found;
			pitch_servo_online_ = true;
		}

		if (!yaw_servo_online_ && !pitch_servo_online_) {
			if (rm_test::platform::drivers::devices::actuators::serial_servo::ReadId(
				    0xFEU, &found, 200U) == 0) {
				yaw_servo_id_ = found;
				yaw_servo_online_ = true;
			}
		}
	}
	return 0;
}

int GimbalModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<GimbalModule, &GimbalModule::RunLoop>(
		&thread_,
		g_gimbal_module_stack,
		K_THREAD_STACK_SIZEOF(g_gimbal_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"gimbal_module");
	started_ = true;
	return 0;
}

void GimbalModule::SetYawAngle(float degrees)
{
	yaw_angle_deg_ = std::clamp(degrees, kYawMin, kYawMax);
}

void GimbalModule::SetPitchAngle(float degrees)
{
	pitch_angle_deg_ = std::clamp(degrees, kPitchMin, kPitchMax);
}

void GimbalModule::HandleCommand(const channels::GimbalCommandMessage &command)
{
	command_enable_ = command.enable;
	idle_ticks_ = 0U;

	if (command.enable == 0U) {
		return;
	}

	SetYawAngle(yaw_angle_deg_ + command.yaw_delta_deg);
	SetPitchAngle(pitch_angle_deg_ + command.pitch_delta_deg);
}

void GimbalModule::RunLoop()
{
	printk("gimbal module started (servo_ready=%d)\n", servo_ready_ ? 1 : 0);

	channels::GimbalCommandMessage command = {};
	channels::GimbalStateMessage state = {};
	while (true) {
		if (zbus_chan_read(&rm_test_gimbal_command_chan, &command, K_NO_WAIT) == 0) {
			HandleCommand(command);
		} else {
			++idle_ticks_;
			if (idle_ticks_ > kNoCommandStopTicks) {
				command_enable_ = 0U;
			}
		}

		if (servo_ready_) {
			if (command_enable_ == 0U) {
				if (!servo_stopped_) {
					if (yaw_servo_online_) {
						(void)rm_test::platform::drivers::devices::actuators::serial_servo::Stop(yaw_servo_id_);
					}
					if (pitch_servo_online_) {
						(void)rm_test::platform::drivers::devices::actuators::serial_servo::Stop(pitch_servo_id_);
					}
					servo_stopped_ = true;
					yaw_sent_once_ = false;
					pitch_sent_once_ = false;
				}
			} else {
				const int64_t now_ms = k_uptime_get();
				if (now_ms >= next_servo_send_ms_) {
					const float yaw_servo_angle = yaw_angle_deg_ + 120.0f;
					const float pitch_servo_angle = pitch_angle_deg_ + 90.0f;

					if (yaw_servo_online_) {
						if (!yaw_sent_once_ ||
						    (std::fabs(yaw_servo_angle - yaw_last_sent_angle_deg_) >=
						     kServoAngleEpsilonDeg)) {
							(void)rm_test::platform::drivers::devices::actuators::serial_servo::MoveToAngle(
								yaw_servo_id_,
								yaw_servo_angle,
								kServoMoveTimeMs);
							yaw_last_sent_angle_deg_ = yaw_servo_angle;
							yaw_sent_once_ = true;
						}
					}

					if (pitch_servo_online_) {
						if (!pitch_sent_once_ ||
						    (std::fabs(pitch_servo_angle - pitch_last_sent_angle_deg_) >=
						     kServoAngleEpsilonDeg)) {
							(void)rm_test::platform::drivers::devices::actuators::serial_servo::MoveToAngle(
								pitch_servo_id_,
								pitch_servo_angle,
								kServoMoveTimeMs);
							pitch_last_sent_angle_deg_ = pitch_servo_angle;
							pitch_sent_once_ = true;
						}
					}

					next_servo_send_ms_ = now_ms + kServoUpdatePeriodMs;
				}
				servo_stopped_ = false;
			}
		}

		state.yaw_angle_deg = yaw_angle_deg_;
		state.pitch_angle_deg = pitch_angle_deg_;
		state.servo_ready = servo_ready_ ? 1U : 0U;
		state.sequence = ++state_sequence_;
		(void)zbus_chan_pub(&rm_test_gimbal_state_chan, &state, K_NO_WAIT);

		if (kBaselineTraceEnabled && ((state_sequence_ % kBaselineTracePeriod) == 0U)) {
			printk("[baseline][gimbal] yaw=%.2f pitch=%.2f en=%u idle=%u yid=%u pid=%u yon=%u pon=%u\n",
			       static_cast<double>(yaw_angle_deg_),
			       static_cast<double>(pitch_angle_deg_),
			       static_cast<unsigned int>(command_enable_),
			       static_cast<unsigned int>(idle_ticks_),
			       static_cast<unsigned int>(yaw_servo_id_),
			       static_cast<unsigned int>(pitch_servo_id_),
			       yaw_servo_online_ ? 1U : 0U,
			       pitch_servo_online_ ? 1U : 0U);
		}
		k_sleep(K_MSEC(2));
	}
}

}  // namespace rm_test::app::modules::gimbal
