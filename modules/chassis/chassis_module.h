/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <zephyr/kernel.h>

#include <modules/chassis/ground_balance_controller.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>
namespace modules::chassis {

class ChassisModule {
public:
	const char *Name() const { return "chassis"; }
	int Initialize();
	int Start();

private:
	static constexpr uint32_t kMitEnterRepeatTicks = 50U;

	void RunLoop();
	void SendDmEnterFrames();
	void SendDmTorqueCommand(uint8_t bus, uint16_t can_id, double torque);

	struct k_thread thread_;
	bool started_ = false;
	uint32_t loop_ticks_ = 0U;

	protocols::motors::dji::DjiMotorFeedback left_wheel_state_;
	protocols::motors::dji::DjiMotorFeedback right_wheel_state_;
	protocols::motors::dm::DmMotorFeedbackNormal left_joint_B_state;
	protocols::motors::dm::DmMotorFeedbackNormal right_joint_B_state;
	protocols::motors::dm::DmMotorFeedbackNormal left_joint_D_state;
	protocols::motors::dm::DmMotorFeedbackNormal right_joint_D_state;
	wbr::v2::GroundBalanceController leg_length_controller_;

};

}  // namespace modules::chassis
