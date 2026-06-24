#ifndef WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_
#define WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_

#include "contact_safety.h"
#include "controller_io.h"
#include "controller_types.h"
#include "leg_kinematics.h"
#include "yaw_coordinator.h"

namespace wbr::v2 {

struct GroundBalanceInput {
  double control_dt = 0.001;
  bool time_reset = false;
  bool observation_valid = false;
  bool yaw_plant_enabled = false;

  LegKinematics leg[2];
  bool leg_valid[2] = {};
  bool wheel_grounded[2] = {};
  double wheel_normal_force[2] = {};

  double roll = 0.0;
  double pitch = 0.0;
  double roll_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;
  double x = 0.0;
  double x_speed = 0.0;
  double wheel_odometry_x_speed = 0.0;
  double wheel_odometry_confidence = 0.0;

  double total_mass = 0.0;
  double gravity_magnitude = 9.81;
  double target_leg_length = 0.18;
  double target_leg_angle = 0.0;
};

struct GroundBalanceOutput {
  control::ControlOutput actuator;
  double sanitized_leg_length = 0.18;
  double sanitized_leg_angle = 0.0;
};

class GroundBalanceController {
 public:
  void Reset();
  void InitializeLegTarget(double length, double angle);
  GroundBalanceOutput Update(const GroundBalanceInput& input);

  void SetLqrEnabled(bool enabled) { lqr_enabled_ = enabled; }
  void SetYawEnabled(bool enabled) { yaw_enabled_ = enabled; }
  void SetVelocityCommand(double linear_velocity, double yaw_rate) {
    target_linear_velocity_ = linear_velocity;
    target_yaw_rate_ = yaw_rate;
  }
  void ResetLqrReference() { lqr_initialized_ = false; }
  const WbrControllerV2Telemetry& telemetry() const { return telemetry_; }

 private:
  double leg_speed_[2] = {};
  double leg_angle_speed_[2] = {};
  double filtered_yaw_speed_ = 0.0;
  double filtered_yaw_acceleration_ = 0.0;
  double previous_yaw_speed_ = 0.0;
  double x_reference_ = 0.0;
  double target_linear_velocity_ = 0.0;
  double target_yaw_rate_ = 0.0;
  double commanded_linear_velocity_ = 0.0;
  YawCoordinator yaw_coordinator_;
  double commanded_yaw_torque_ = 0.0;
  double commanded_differential_leg_angle_torque_ = 0.0;
  double commanded_leg_length_ = 0.18;
  double commanded_leg_angle_ = 0.0;
  double support_factor_[2] = {};
  double leg_length_integral_[2] = {};
  ContactSafetyMachine contact_safety_;
  double contact_leg_length_offset_[2] = {};
  bool command_initialized_ = false;
  bool lqr_initialized_ = false;
  bool lqr_enabled_ = true;
  bool yaw_enabled_ = true;
  WbrContactSafetyState contact_safety_state_ =
      WbrContactSafetyState::kAirborne;
  WbrControllerV2Telemetry telemetry_{};
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_
