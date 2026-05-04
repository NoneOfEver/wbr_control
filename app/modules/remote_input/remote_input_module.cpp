/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/sys/printk.h>

#include <app/modules/remote_input/remote_input_module.h>
#include <app/bootstrap/thread_utils.h>
#include <app/protocols/remote_input/dr16_protocol.h>
#include <app/protocols/remote_input/vt03_protocol.h>

namespace {

K_THREAD_STACK_DEFINE(g_remote_input_module_stack, 1024);

}  // namespace

namespace rm_test::app::modules::remote_input {

int RemoteInputModule::Initialize()
{
	publish_sequence_ = 0U;
	started_ = false;
	latest_input_ = {};
	latest_input_valid_ = false;
	line_pos_ = 0U;
	line_buf_[0] = '\0';
	binary_len_ = 0U;
	return 0;
}

int RemoteInputModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<RemoteInputModule, &RemoteInputModule::RunLoop>(
		&thread_,
		g_remote_input_module_stack,
		K_THREAD_STACK_SIZEOF(g_remote_input_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"remote_input_module");

	started_ = true;
	return 0;
}

void RemoteInputModule::RunLoop()
{
	printk("remote_input module started\n");

	channels::ChassisCommandMessage chassis_command = {};
	channels::ArmCommandMessage arm_command = {};
	channels::GimbalCommandMessage gimbal_command = {};
	channels::GantryCommandMessage gantry_command = {};

	while (true) {
		DecodeUartFramesInQueue();

		if (latest_input_valid_) {
			chassis_command = ComposeChassisCommand(latest_input_);
			arm_command = ComposeArmCommand(latest_input_);
			gimbal_command = ComposeGimbalCommand(latest_input_);
			gantry_command = ComposeGantryCommand(latest_input_);
		} else {
			chassis_command = {};
			arm_command = {};
			gimbal_command = {};
			gantry_command = {};
			chassis_command.source = channels::kInputSourceUnknown;
			arm_command.source = channels::kInputSourceUnknown;
			gimbal_command.source = channels::kInputSourceUnknown;
			gantry_command.source = channels::kInputSourceUnknown;
		}

		const uint32_t sequence = ++publish_sequence_;
		chassis_command.sequence = sequence;
		arm_command.sequence = sequence;
		gimbal_command.sequence = sequence;
		gantry_command.sequence = sequence;

		(void)zbus_chan_pub(&rm_test_chassis_command_chan, &chassis_command, K_NO_WAIT);
		(void)zbus_chan_pub(&rm_test_arm_command_chan, &arm_command, K_NO_WAIT);
		(void)zbus_chan_pub(&rm_test_gimbal_command_chan, &gimbal_command, K_NO_WAIT);
		(void)zbus_chan_pub(&rm_test_gantry_command_chan, &gantry_command, K_NO_WAIT);
		k_sleep(K_MSEC(1));
	}
}

int RemoteInputModule::ParseLine(const char *line, channels::RemoteInputMessage *out)
{
	if ((line == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	char type[8] = {0};
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
	int en = 0;

	if (sscanf(line, "%7s %f %f %f %d", type, &a, &b, &c, &en) != 5) {
		return -EINVAL;
	}

	if (strcmp(type, "dr16") == 0) {
		out->source = channels::kRemoteInputDr16;
		out->axis_lx = a;
		out->axis_ly = b;
		out->axis_wheel = c;
		out->axis_jx = 0.0f;
		out->axis_jy = 0.0f;
		out->axis_jz = 0.0f;
	} else if (strcmp(type, "vt03") == 0) {
		out->source = channels::kRemoteInputVt03;
		out->axis_jx = a;
		out->axis_jy = b;
		out->axis_jz = c;
		out->axis_lx = 0.0f;
		out->axis_ly = 0.0f;
		out->axis_wheel = 0.0f;
	} else {
		return -EINVAL;
	}

	out->chassis_enable = (en != 0) ? 1U : 0U;
	out->sequence = ++publish_sequence_;
	return 0;
}

void RemoteInputModule::ConsumeBinary(size_t bytes)
{
	if ((bytes == 0U) || (binary_len_ == 0U)) {
		return;
	}

	if (bytes >= binary_len_) {
		binary_len_ = 0U;
		return;
	}

	memmove(binary_buf_, binary_buf_ + bytes, binary_len_ - bytes);
	binary_len_ -= bytes;
}

void RemoteInputModule::TryDecodeBinaryFrames()
{
	while (binary_len_ > 0U) {
		channels::RemoteInputMessage input = {};

		if ((binary_len_ >= protocols::remote_input::vt03::kRemoteFrameLength) &&
		    (binary_buf_[0] == 0xa9U) && (binary_buf_[1] == 0x53U)) {
			protocols::remote_input::vt03::Vt03Frame vt03_frame = {};
			if (protocols::remote_input::vt03::DecodeRemoteFrame(binary_buf_, binary_len_, &vt03_frame)) {
				input.source = channels::kRemoteInputVt03;
				input.axis_jx = vt03_frame.left_y;
				input.axis_jy = vt03_frame.left_x;
				input.axis_jz = vt03_frame.wheel;
				input.chassis_enable = vt03_frame.chassis_enable ? 1U : 0U;
				input.sequence = ++publish_sequence_;
				latest_input_ = input;
				latest_input_valid_ = true;
				(void)zbus_chan_pub(&rm_test_remote_input_chan, &input, K_NO_WAIT);
				ConsumeBinary(protocols::remote_input::vt03::kRemoteFrameLength);
				continue;
			}
			ConsumeBinary(1U);
			continue;
		}

		if ((binary_len_ >= protocols::remote_input::vt03::kCustomFrameLength) &&
		    (binary_buf_[0] == 0xa5U)) {
			protocols::remote_input::vt03::Vt03CustomFrame custom = {};
			if (protocols::remote_input::vt03::DecodeCustomFrame(binary_buf_, binary_len_, &custom)) {
				input.source = channels::kRemoteInputVt03;
				input.axis_jx = custom.joystick_x;
				input.axis_jy = custom.joystick_y;
				input.axis_jz = custom.joystick_z;
				input.chassis_enable = custom.chassis_enable ? 1U : 0U;
				input.sequence = ++publish_sequence_;
				latest_input_ = input;
				latest_input_valid_ = true;
				(void)zbus_chan_pub(&rm_test_remote_input_chan, &input, K_NO_WAIT);
				ConsumeBinary(protocols::remote_input::vt03::kCustomFrameLength);
				continue;
			}
		}

		if (binary_len_ >= protocols::remote_input::dr16::kFrameLength) {
			protocols::remote_input::dr16::Dr16Frame dr16_frame = {};
			if (protocols::remote_input::dr16::DecodeFrame(binary_buf_, binary_len_, &dr16_frame)) {
				input.source = channels::kRemoteInputDr16;
				input.axis_lx = dr16_frame.left_stick_x;
				input.axis_ly = dr16_frame.left_stick_y;
				input.axis_wheel = dr16_frame.wheel;
				input.chassis_enable = dr16_frame.chassis_enable ? 1U : 0U;
				input.sequence = ++publish_sequence_;
				latest_input_ = input;
				latest_input_valid_ = true;
				(void)zbus_chan_pub(&rm_test_remote_input_chan, &input, K_NO_WAIT);
				ConsumeBinary(protocols::remote_input::dr16::kFrameLength);
				continue;
			}
		}

		if (binary_len_ < protocols::remote_input::dr16::kFrameLength) {
			break;
		}

		ConsumeBinary(1U);
	}
}

void RemoteInputModule::DecodeUartFramesInQueue()
{
	while (true) {
		channels::uart_raw_frame_queue::UartRawFrameMessage frame = {};
		if (channels::uart_raw_frame_queue::DequeueForRemoteInput(&frame) != 0) {
			break;
		}

		for (uint8_t i = 0U; i < frame.len; ++i) {
			const uint8_t byte = frame.data[i];

			if (binary_len_ < kBinaryBufSize) {
				binary_buf_[binary_len_++] = byte;
			} else {
				memmove(binary_buf_, binary_buf_ + 1, kBinaryBufSize - 1U);
				binary_buf_[kBinaryBufSize - 1U] = byte;
			}

			TryDecodeBinaryFrames();

			if (byte == '\r') {
				continue;
			}

			if (byte == '\n') {
				line_buf_[line_pos_] = '\0';
				if (line_pos_ > 0U) {
					channels::RemoteInputMessage input = {};
					if (ParseLine(line_buf_, &input) == 0) {
						latest_input_ = input;
						latest_input_valid_ = true;
						(void)zbus_chan_pub(&rm_test_remote_input_chan, &input, K_NO_WAIT);
					}
				}
				line_pos_ = 0U;
				line_buf_[0] = '\0';
				continue;
			}

			if (isprint(byte) == 0) {
				continue;
			}

			if (line_pos_ < (kLineBufSize - 1U)) {
				line_buf_[line_pos_++] = static_cast<char>(byte);
			} else {
				line_pos_ = 0U;
				line_buf_[0] = '\0';
			}
		}
	}
}

channels::ChassisCommandMessage RemoteInputModule::ComposeChassisCommand(
	const channels::RemoteInputMessage &input) const
{
	channels::ChassisCommandMessage command = {};

	if (input.chassis_enable == 0U) {
		command.source = channels::kInputSourceUnknown;
		return command;
	}

	switch (input.source) {
	case channels::kRemoteInputDr16:
		command.target_vx = input.axis_lx * kChassisSpeedScale;
		command.target_vy = -input.axis_ly * kChassisSpeedScale;
		command.target_wz = input.axis_wheel * kChassisSpeedScale;
		command.source = channels::kInputSourceDr16;
		break;
	case channels::kRemoteInputVt03:
		command.target_vx = -input.axis_jx * kChassisSpeedScale;
		command.target_vy = input.axis_jy * kChassisSpeedScale;
		command.target_wz = input.axis_jz * kChassisSpeedScale * kVt03SpinScale;
		command.source = channels::kInputSourceVt03;
		break;
	default:
		command.source = channels::kInputSourceUnknown;
		break;
	}

	return command;
}

channels::ArmCommandMessage RemoteInputModule::ComposeArmCommand(
	const channels::RemoteInputMessage &input) const
{
	channels::ArmCommandMessage command = {};

	if (input.chassis_enable == 0U) {
		command.source = channels::kInputSourceUnknown;
		return command;
	}

	switch (input.source) {
	case channels::kRemoteInputDr16:
		command.claw_delta = input.axis_wheel * 0.003f;
		command.elbow_yaw_delta = input.axis_lx * 0.003f;
		command.elbow_pitch_delta = input.axis_ly * 0.003f;
		command.wrist_twist_delta = 0.0f;
		command.wrist_flip_delta = 0.0f;
		command.source = channels::kInputSourceDr16;
		command.enable = 1U;
		break;
	case channels::kRemoteInputVt03:
		command.claw_delta = 0.0f;
		command.elbow_yaw_delta = input.axis_jx * 0.003f;
		command.elbow_pitch_delta = 0.0f;
		command.wrist_twist_delta = input.axis_jz * 0.003f;
		command.wrist_flip_delta = input.axis_jy * 0.003f;
		command.source = channels::kInputSourceVt03;
		command.enable = 1U;
		break;
	default:
		command.source = channels::kInputSourceUnknown;
		break;
	}

	return command;
}

channels::GimbalCommandMessage RemoteInputModule::ComposeGimbalCommand(
	const channels::RemoteInputMessage &input) const
{
	channels::GimbalCommandMessage command = {};

	if ((input.chassis_enable == 0U) || (input.source != channels::kRemoteInputVt03)) {
		command.source = channels::kInputSourceUnknown;
		return command;
	}

	command.yaw_delta_deg = input.axis_jx * 1.2f;
	command.pitch_delta_deg = input.axis_jy * 0.8f;
	command.source = channels::kInputSourceVt03;
	command.enable = 1U;
	return command;
}

channels::GantryCommandMessage RemoteInputModule::ComposeGantryCommand(
	const channels::RemoteInputMessage &input) const
{
	channels::GantryCommandMessage command = {};

	if (input.chassis_enable == 0U) {
		command.source = channels::kInputSourceUnknown;
		return command;
	}

	switch (input.source) {
	case channels::kRemoteInputDr16:
		command.x_delta = input.axis_lx * 0.02f;
		command.y_delta = input.axis_ly * 0.02f;
		command.z_delta = input.axis_wheel * 0.01f;
		command.source = channels::kInputSourceDr16;
		command.enable = 1U;
		break;
	case channels::kRemoteInputVt03:
		command.x_delta = input.axis_jx * 0.02f;
		command.y_delta = input.axis_jy * 0.02f;
		command.z_delta = input.axis_jz * 0.01f;
		command.source = channels::kInputSourceVt03;
		command.enable = 1U;
		break;
	default:
		command.source = channels::kInputSourceUnknown;
		break;
	}

	return command;
}

}  // namespace rm_test::app::modules::remote_input
