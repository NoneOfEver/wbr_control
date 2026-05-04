/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_MODULES_GIMBAL_GIMBAL_MODULE_H_
#define RM_TEST_APP_MODULES_GIMBAL_GIMBAL_MODULE_H_

#include <zephyr/kernel.h>

#include <app/bootstrap/module.h>
#include <app/channels/gimbal_command_channel.h>
#include <app/channels/gimbal_state_channel.h>

namespace rm_test::app::modules::gimbal {

class GimbalModule : public bootstrap::Module {
public:
	const char *Name() const override { return "gimbal"; }
	int Initialize() override;
	int Start() override;

	void SetYawAngle(float degrees);
	void SetPitchAngle(float degrees);

private:
	static constexpr float kYawMin = -180.0f;
	static constexpr float kYawMax = 180.0f;
	static constexpr float kPitchMin = -30.0f;
	static constexpr float kPitchMax = 45.0f;
	static constexpr uint8_t kDefaultYawServoId = 1U;
	static constexpr uint8_t kDefaultPitchServoId = 2U;
	static constexpr uint16_t kServoMoveTimeMs = 20U;
	static constexpr uint16_t kServoUpdatePeriodMs = 20U;
	static constexpr float kServoAngleEpsilonDeg = 0.2f;
	static constexpr uint32_t kNoCommandStopTicks = 100U;

	void RunLoop();
	void HandleCommand(const channels::GimbalCommandMessage &command);

	struct k_thread thread_;
	bool started_ = false;
	bool servo_ready_ = false;
	bool servo_stopped_ = false;
	bool yaw_servo_online_ = false;
	bool pitch_servo_online_ = false;
	uint8_t command_enable_ = 0U;
	uint32_t idle_ticks_ = 0U;
	uint8_t yaw_servo_id_ = kDefaultYawServoId;
	uint8_t pitch_servo_id_ = kDefaultPitchServoId;
	float yaw_angle_deg_ = 0.0f;
	float pitch_angle_deg_ = 0.0f;
	float yaw_last_sent_angle_deg_ = 0.0f;
	float pitch_last_sent_angle_deg_ = 0.0f;
	bool yaw_sent_once_ = false;
	bool pitch_sent_once_ = false;
	int64_t next_servo_send_ms_ = 0;
	uint32_t state_sequence_ = 0U;
};

}  // namespace rm_test::app::modules::gimbal

#endif /* RM_TEST_APP_MODULES_GIMBAL_GIMBAL_MODULE_H_ */
