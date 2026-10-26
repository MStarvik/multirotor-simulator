#pragma once

#include <cstdint>
#include <optional>

#include "sim/sensors.hpp"
#include "sim/simulation.hpp"
#include "sim/vehicle_state.hpp"

namespace sim::runtime {

// Normalized stick positions, each in [-1, 1].
struct PilotInput {
  double roll = 0.0;      // +: right.
  double pitch = 0.0;     // +: forward (nose down).
  double yaw = 0.0;       // +: clockwise seen from above.
  double throttle = 0.0;  // +: up.

  bool operator==(const PilotInput&) const = default;
};

// What a flight controller gets each step.
struct ControllerInput {
  std::uint64_t step;
  double time;       // s
  double step_size;  // s
  const SensorReadings& sensors;
  // The true state. Built-in controllers may use it; an external flight controller
  // gets it for logging and estimator evaluation only.
  const VehicleState& truth;
};

// Turns sensor readings into motor commands, once per simulation step.
class FlightController {
 public:
  virtual ~FlightController() = default;

  // The simulation was reset.
  virtual void reset() = 0;
  virtual void set_pilot_input(const PilotInput& input) = 0;

  // Returns the motor commands for the next step, or nullopt if they aren't ready
  // yet. Then the simulation doesn't advance, and update() is called again with the
  // same input. This is how an external flight controller holds the simulation in
  // lockstep with itself; implementations should wait briefly rather than spin.
  [[nodiscard]] virtual std::optional<MotorCommands> update(const ControllerInput& input) = 0;
};

}  // namespace sim::runtime
