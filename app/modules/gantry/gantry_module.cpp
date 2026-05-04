/* SPDX-License-Identifier: Apache-2.0 */

#include <algorithm>

#include <math.h>

#include <zephyr/sys/printk.h>

#include <app/bootstrap/thread_utils.h>
#include <app/modules/gantry/gantry_module.h>
#include <app/protocols/motors/cubemars_motor_protocol.h>
#include <app/protocols/motors/dji_motor_protocol.h>
#include <app/services/actuator/actuator_service.h>
#include <platform/drivers/communication/can_dispatch.h>

namespace {

K_THREAD_STACK_DEFINE(g_gantry_module_stack, 1024);
constexpr bool kBaselineTraceEnabled = false;
constexpr uint32_t kBaselineTracePeriod = 500U;
constexpr int kMaxFramesPerTick = 8;

float UIntToFloat(uint16_t raw, float min_value, float max_value, uint8_t bits)
{
	const float span = max_value - min_value;
	const float scale = static_cast<float>((1U << bits) - 1U);
	return (static_cast<float>(raw) * span / scale) + min_value;
}

}  // namespace

namespace rm_test::app::modules::gantry {

int GantryModule::Initialize()
{
	started_ = false;
	cubemars_started_ = false;
	x_axis_virtual_distance_ = 0.0f;
	y_axis_virtual_distance_ = 0.0f;
	z_axis_virtual_distance_ = 0.0f;
	x_left_feedback_ = {};
	x_right_feedback_ = {};
	y_feedback_ = {};
	z_left_state_ = {};
	z_right_state_ = {};

	x_left_speed_pid_.Init(kDjiSpeedPidKp,
			 kDjiSpeedPidKi,
			 kDjiSpeedPidKd,
			 0.0f,
			 kDjiCurrentLimit,
			 kDjiCurrentLimit,
			 0.002f);
	x_right_speed_pid_.Init(kDjiSpeedPidKp,
			  kDjiSpeedPidKi,
			  kDjiSpeedPidKd,
			  0.0f,
			  kDjiCurrentLimit,
			  kDjiCurrentLimit,
			  0.002f);
	y_speed_pid_.Init(kDjiSpeedPidKp,
			 kDjiSpeedPidKi,
			 kDjiSpeedPidKd,
			 0.0f,
			 kDjiCurrentLimit,
			 kDjiCurrentLimit,
			 0.002f);
	z_left_pid_angle_.Init(20.0f, 0.1f, 0.1f, 0.0f, 29.0f, 50.0f, 0.01f);
	z_right_pid_angle_.Init(20.0f, 0.1f, 0.1f, 0.0f, 29.0f, 50.0f, 0.01f);
	return 0;
}

int GantryModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<GantryModule, &GantryModule::RunLoop>(
		&thread_,
		g_gantry_module_stack,
		K_THREAD_STACK_SIZEOF(g_gantry_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"gantry_module");
	started_ = true;
	return 0;
}

void GantryModule::XAxisMoveInDistance(float distance)
{
	x_axis_virtual_distance_ = std::clamp(distance, -kXAxisLimit, kXAxisLimit);
}

void GantryModule::YAxisMoveInDistance(float distance)
{
	y_axis_virtual_distance_ = std::clamp(distance, -kYAxisLimit, kYAxisLimit);
}

void GantryModule::ZAxisMoveInDistance(float distance)
{
	z_axis_virtual_distance_ = std::clamp(distance, -kZAxisLimit, kZAxisLimit);
}

void GantryModule::HandleCommand(const channels::GantryCommandMessage &command)
{
	if (command.enable == 0U) {
		x_axis_virtual_distance_ = 0.0f;
		y_axis_virtual_distance_ = 0.0f;
		z_axis_virtual_distance_ = 0.0f;
		return;
	}

	XAxisMoveInDistance(x_axis_virtual_distance_ + command.x_delta);
	YAxisMoveInDistance(y_axis_virtual_distance_ + command.y_delta);
	ZAxisMoveInDistance(z_axis_virtual_distance_ + command.z_delta);
}

void GantryModule::SendCubemarsStartupSequence()
{
	uint8_t frame[8] = {0U};
	if (rm_test::app::protocols::motors::cubemars::GetSaveZeroFrame(frame) == 0) {
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan2,
			kCubemarsCanId,
			frame,
			8U);
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan1,
			kCubemarsCanId,
			frame,
			8U);
		k_sleep(K_MSEC(200));
	}

	if (rm_test::app::protocols::motors::cubemars::GetEnterFrame(frame) == 0) {
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan2,
			kCubemarsCanId,
			frame,
			8U);
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan1,
			kCubemarsCanId,
			frame,
			8U);
		k_sleep(K_MSEC(200));
	}
}

void GantryModule::DecodeCanFramesInQueue()
{
	constexpr int kMaxFramesPerTick = 8;
	int frames_processed = 0;
	while (frames_processed < kMaxFramesPerTick) {
		rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage frame = {};
		if (rm_test::app::channels::can_raw_frame_queue::DequeueForGantry(&frame) != 0) {
			break;
		}

		if ((frame.bus == 2U) && ((frame.can_id == kDjiXLeftCanId) || (frame.can_id == kDjiXRightCanId))) {
			rm_test::app::protocols::motors::dji::DjiMotorFeedback dji = {};
			if (rm_test::app::protocols::motors::dji::DecodeFeedback(frame.data, frame.dlc, &dji) == 0) {
				if (frame.can_id == kDjiXLeftCanId) {
					x_left_feedback_.omega = dji.omega;
					x_left_feedback_.valid = true;
				} else {
					x_right_feedback_.omega = dji.omega;
					x_right_feedback_.valid = true;
				}
			}
			++frames_processed;
		}

		if ((frame.bus == 3U) && (frame.can_id == kDjiYCanId)) {
			rm_test::app::protocols::motors::dji::DjiMotorFeedback dji = {};
			if (rm_test::app::protocols::motors::dji::DecodeFeedback(frame.data, frame.dlc, &dji) == 0) {
				y_feedback_.omega = dji.omega;
				y_feedback_.valid = true;
			}
			++frames_processed;
		}

		if (frame.can_id == kCubemarsCanId) {
			rm_test::app::protocols::motors::cubemars::CubemarsFeedback z = {};
			if (rm_test::app::protocols::motors::cubemars::DecodeFeedback(frame.data, frame.dlc, &z) != 0) {
				continue;
			}

			CubemarsState *state = nullptr;
			if (frame.bus == 2U) {
				state = &z_left_state_;
			} else if (frame.bus == 1U) {
				state = &z_right_state_;
			}

			if (state != nullptr) {
				state->angle = UIntToFloat(z.position_raw, -12.5f, 12.5f, 16);
				state->omega = UIntToFloat(z.velocity_raw, -50.0f, 50.0f, 12);
				state->torque = UIntToFloat(z.torque_raw, -18.0f, 18.0f, 12);
				state->valid = true;
			}
		}
		frames_processed++;
	}
}

void GantryModule::ApplyControlAndSend()
{
	const float x_target_omega = std::clamp(x_axis_virtual_distance_ * kDistanceToSpeedKp,
					      -kXAxisSpeedLimit,
					      kXAxisSpeedLimit);
	const float y_target_omega = std::clamp(y_axis_virtual_distance_ * kDistanceToSpeedKp,
					      -kYAxisSpeedLimit,
					      kYAxisSpeedLimit);

	int16_t can2_current[4] = {0, 0, 0, 0};
	if (x_left_feedback_.valid) {
		can2_current[0] = static_cast<int16_t>(
			x_left_speed_pid_.update(x_target_omega, static_cast<float>(x_left_feedback_.omega)));
	}
	if (x_right_feedback_.valid) {
		can2_current[1] = static_cast<int16_t>(
			x_right_speed_pid_.update(-x_target_omega, static_cast<float>(x_right_feedback_.omega)));
	}
	(void)rm_test::app::services::actuator::SendMotorCurrent(
		rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan2,
		rm_test::app::services::actuator::MotorCurrentGroup::kDji0x200,
		can2_current);

	int16_t can3_current[4] = {0, 0, 0, 0};
	if (y_feedback_.valid) {
		can3_current[0] = static_cast<int16_t>(
			y_speed_pid_.update(y_target_omega, static_cast<float>(y_feedback_.omega)));
	}
	(void)rm_test::app::services::actuator::SendMotorCurrent(
		rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan3,
		rm_test::app::services::actuator::MotorCurrentGroup::kDji0x200,
		can3_current);

	const float z_target_left_angle = -(z_axis_virtual_distance_ * kZAxisRadPerDistance);
	const float z_target_right_angle = +(z_axis_virtual_distance_ * kZAxisRadPerDistance);
	const float z_left_omega = z_left_state_.valid
					 ? std::clamp(z_left_pid_angle_.update(z_target_left_angle, z_left_state_.angle),
						      -kZAxisSpeedLimit,
						      kZAxisSpeedLimit)
					 : 0.0f;
	const float z_right_omega = z_right_state_.valid
					  ? std::clamp(z_right_pid_angle_.update(z_target_right_angle, z_right_state_.angle),
						       -kZAxisSpeedLimit,
						       kZAxisSpeedLimit)
					  : 0.0f;

	rm_test::app::protocols::motors::cubemars::CubemarsMitRange z_range = {
		.p_min = -12.5f,
		.p_max = 12.5f,
		.v_min = -50.0f,
		.v_max = 50.0f,
		.kp_min = 0.0f,
		.kp_max = 500.0f,
		.kd_min = 0.0f,
		.kd_max = 5.0f,
		.t_min = -18.0f,
		.t_max = 18.0f,
	};

	rm_test::app::protocols::motors::cubemars::CubemarsMitCommand left_cmd = {
		.position = 0.0f,
		.velocity = z_left_omega,
		.kp = 0.0f,
		.kd = kCubemarsZKd,
		.torque = 0.0f,
	};
	rm_test::app::protocols::motors::cubemars::CubemarsMitCommand right_cmd = {
		.position = 0.0f,
		.velocity = z_right_omega,
		.kp = 0.0f,
		.kd = kCubemarsZKd,
		.torque = 0.0f,
	};

	uint8_t z_frame[8] = {0U};
	if (rm_test::app::protocols::motors::cubemars::PackMitCommand(&left_cmd, &z_range, z_frame) == 0) {
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan2,
			kCubemarsCanId,
			z_frame,
			8U);
	}
	if (rm_test::app::protocols::motors::cubemars::PackMitCommand(&right_cmd, &z_range, z_frame) == 0) {
		(void)rm_test::platform::drivers::communication::can_dispatch::SendStdDataOnBus(
			rm_test::platform::drivers::communication::can_dispatch::CanBus::kCan1,
			kCubemarsCanId,
			z_frame,
			8U);
	}
}

void GantryModule::RunLoop()
{
	printk("gantry module started\n");
	if (!cubemars_started_) {
		SendCubemarsStartupSequence();
		cubemars_started_ = true;
	}

	channels::GantryCommandMessage command = {};
	uint32_t tick = 0U;
	while (true) {
		DecodeCanFramesInQueue();

		bool has_command = false;
		if (zbus_chan_read(&rm_test_gantry_command_chan, &command, K_NO_WAIT) == 0) {
			HandleCommand(command);
			has_command = true;
		}

		if (command.enable == 0U) {
			if (kBaselineTraceEnabled && ((++tick % kBaselineTracePeriod) == 0U)) {
				printk("[baseline][gantry] virt=(%.3f %.3f %.3f) x=(%d %d) y=%d z=(%.2f %.2f) en=%u\n",
				       static_cast<double>(x_axis_virtual_distance_),
				       static_cast<double>(y_axis_virtual_distance_),
				       static_cast<double>(z_axis_virtual_distance_),
				       static_cast<int>(x_left_feedback_.omega),
				       static_cast<int>(x_right_feedback_.omega),
				       static_cast<int>(y_feedback_.omega),
				       static_cast<double>(z_left_state_.angle),
				       static_cast<double>(z_right_state_.angle),
				       static_cast<unsigned int>(command.enable));
			}
			k_sleep(K_MSEC(10));
			continue;
		}

		ApplyControlAndSend();

		if (kBaselineTraceEnabled && ((++tick % kBaselineTracePeriod) == 0U)) {
			printk("[baseline][gantry] virt=(%.3f %.3f %.3f) x=(%d %d) y=%d z=(%.2f %.2f) en=%u\n",
			       static_cast<double>(x_axis_virtual_distance_),
			       static_cast<double>(y_axis_virtual_distance_),
			       static_cast<double>(z_axis_virtual_distance_),
			       static_cast<int>(x_left_feedback_.omega),
			       static_cast<int>(x_right_feedback_.omega),
			       static_cast<int>(y_feedback_.omega),
			       static_cast<double>(z_left_state_.angle),
			       static_cast<double>(z_right_state_.angle),
			       static_cast<unsigned int>(command.enable));
		}
		k_sleep(K_MSEC(2));
	}
}

}  // namespace rm_test::app::modules::gantry
