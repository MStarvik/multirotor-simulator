#include "control/attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace sim::control {

AttitudeController::AttitudeController(const MultirotorModel& model, AttitudeControllerGains gains)
    : body_{model.body}, mixer_{model}, gains_{gains} {}

void AttitudeController::reset() {
  pilot_ = {};
  yaw_setpoint_ = 0.0;
}

std::optional<MotorCommands> AttitudeController::update(const runtime::ControllerInput& input) {
  const VehicleState& state = input.truth;
  const double dt = input.step_size;

  // Idle on the ground until the pilot asks to climb.
  if (state.on_ground && pilot_.throttle <= 0.0) {
    yaw_setpoint_ = to_euler(state.attitude).yaw;
    return MotorCommands{};
  }

  // Collective thrust that tracks the commanded climb rate, compensated for tilt.
  // The world frame is NED, so climbing is negative z velocity.
  const double climb_rate = pilot_.throttle * gains_.max_climb_rate;
  const double down_acceleration = gains_.climb * (-climb_rate - state.velocity.z);
  const Vec3 body_up = rotate(state.attitude, {0.0, 0.0, -1.0});
  const double tilt_cosine = std::max(-body_up.z, 0.5);
  const double thrust = body_.mass * (gravity - down_acceleration) / tilt_cosine;

  // Attitude: roll and pitch angles from the sticks, yaw by integrating the stick rate.
  yaw_setpoint_ = std::remainder(yaw_setpoint_ + pilot_.yaw * gains_.max_yaw_rate * dt,
                                 2.0 * std::numbers::pi);
  const Quat target = from_euler({
      .roll = pilot_.roll * gains_.max_tilt,
      .pitch = -pilot_.pitch * gains_.max_tilt,  // Stick forward pitches the nose down.
      .yaw = yaw_setpoint_,
  });
  Quat error = conjugate(state.attitude) * target;  // In the body frame.
  if (error.w < 0.0) {
    error = -error;  // Take the short way around.
  }
  const Vec3 rate_setpoint = 2.0 * gains_.attitude * Vec3{error.x, error.y, error.z};

  // Rates: torque for the desired angular acceleration, plus the gyroscopic term.
  const Vec3& omega = state.angular_velocity;
  const Vec3 torque = cwise_mul(body_.inertia, gains_.rate * (rate_setpoint - omega)) +
                      cross(omega, cwise_mul(body_.inertia, omega));

  return mixer_.mix(thrust, torque);
}

}  // namespace sim::control
