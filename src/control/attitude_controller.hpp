#pragma once

#include "control/mixer.hpp"
#include "runtime/flight_controller.hpp"

namespace sim::control {

struct AttitudeControllerGains {
  double max_tilt = 0.5;        // rad, at full roll or pitch stick.
  double max_yaw_rate = 2.0;    // rad/s, at full yaw stick.
  double max_climb_rate = 2.0;  // m/s, at full throttle stick.
  double attitude = 6.0;        // Attitude error to rate setpoint, 1/s.
  double rate = 25.0;           // Rate error to angular acceleration, 1/s.
  double climb = 4.0;           // Climb rate error to vertical acceleration, 1/s.
};

// A minimal flight controller, flown by multirotor-sim-pilot over the SITL socket.
// Sticks set the roll and pitch angles, yaw rate and climb rate. It reads the true
// vehicle state; a real flight controller would only see sensors.
class AttitudeController final : public runtime::FlightController {
 public:
  explicit AttitudeController(const MultirotorModel& model, AttitudeControllerGains gains = {});

  void reset() override;
  void set_pilot_input(const runtime::PilotInput& input) override { pilot_ = input; }
  [[nodiscard]] std::optional<MotorCommands> update(const runtime::ControllerInput& input) override;

 private:
  BodyParams body_;
  Mixer mixer_;
  AttitudeControllerGains gains_;
  runtime::PilotInput pilot_;
  double yaw_setpoint_ = 0.0;
};

}  // namespace sim::control
