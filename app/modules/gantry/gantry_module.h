/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_MODULES_GANTRY_GANTRY_MODULE_H_
#define RM_TEST_APP_MODULES_GANTRY_GANTRY_MODULE_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include <app/algorithms/control/alg_pid.h>
#include <app/bootstrap/module.h>
#include <app/channels/can_raw_frame_queue.h>
#include <app/channels/gantry_command_channel.h>

namespace rm_test::app::modules::gantry {

class GantryModule : public bootstrap::Module {
public:
	const char *Name() const override { return "gantry"; }
	int Initialize() override;
	int Start() override;

	void XAxisMoveInDistance(float distance);
	void YAxisMoveInDistance(float distance);
	void ZAxisMoveInDistance(float distance);

private:
	static constexpr float kXAxisLimit = 10.0f;
	static constexpr float kYAxisLimit = 10.0f;
	static constexpr float kZAxisLimit = 5.0f;
	static constexpr float kXAxisSpeedLimit = 20.0f;
	static constexpr float kYAxisSpeedLimit = 20.0f;
	static constexpr float kZAxisSpeedLimit = 20.0f;
	static constexpr float kZAxisRadPerDistance = 1.0f;
	static constexpr float kDjiSpeedPidKp = 1.0f;
	static constexpr float kDjiSpeedPidKi = 0.0f;
	static constexpr float kDjiSpeedPidKd = 0.0f;
	static constexpr float kDjiCurrentLimit = 10000.0f;
	static constexpr float kDistanceToSpeedKp = 5.0f;
	static constexpr float kCubemarsZKd = 0.8f;
	static constexpr uint16_t kDjiXLeftCanId = 0x201;
	static constexpr uint16_t kDjiXRightCanId = 0x202;
	static constexpr uint16_t kDjiYCanId = 0x201;
	static constexpr uint16_t kCubemarsCanId = 0x000;

	void RunLoop();
	void HandleCommand(const channels::GantryCommandMessage &command);
	void DecodeCanFramesInQueue();
	void ApplyControlAndSend();
	void SendCubemarsStartupSequence();

	struct DjiFeedbackSimple {
		int16_t omega = 0;
		bool valid = false;
	};

	struct CubemarsState {
		float angle = 0.0f;
		float omega = 0.0f;
		float torque = 0.0f;
		bool valid = false;
	};

	struct k_thread thread_;
	bool started_ = false;
	bool cubemars_started_ = false;
	float x_axis_virtual_distance_ = 0.0f;
	float y_axis_virtual_distance_ = 0.0f;
	float z_axis_virtual_distance_ = 0.0f;
	alg::Pid x_left_speed_pid_;
	alg::Pid x_right_speed_pid_;
	alg::Pid y_speed_pid_;
	alg::Pid z_left_pid_angle_;
	alg::Pid z_right_pid_angle_;
	DjiFeedbackSimple x_left_feedback_;
	DjiFeedbackSimple x_right_feedback_;
	DjiFeedbackSimple y_feedback_;
	CubemarsState z_left_state_;
	CubemarsState z_right_state_;
};

}  // namespace rm_test::app::modules::gantry

#endif /* RM_TEST_APP_MODULES_GANTRY_GANTRY_MODULE_H_ */
