#pragma once

#include <array>
#include <cstdint>

#include "sim/math.hpp"
#include "sim/multirotor.hpp"
#include "sim/sensors.hpp"
#include "sim/vehicle_state.hpp"

namespace sim {

// Normalized speed command per rotor, in [0, 1].
using MotorCommands = std::array<double, max_rotors>;

// Rigid-body dynamics of a multirotor, advanced in fixed steps.
//
// This is the whole simulation: it knows nothing about wall-clock time, threads,
// or rendering, so it runs identically headless, in a viewer, or in tests.
class Simulation {
 public:
  // The seed drives all sensor noise, so equal seeds and inputs give equal runs.
  explicit Simulation(MultirotorModel model, double step_size = 0.001, std::uint64_t seed = 1);

  // Commands applied from the next step on, until replaced.
  void set_motor_commands(const MotorCommands& commands) { commands_ = commands; }

  void step();
  void reset(const VehicleState& initial_state = {});

  [[nodiscard]] std::uint64_t step_count() const { return steps_; }
  [[nodiscard]] double time() const { return static_cast<double>(steps_) * step_size_; }
  [[nodiscard]] double step_size() const { return step_size_; }
  [[nodiscard]] const VehicleState& state() const { return state_; }
  // Measurements of the current state.
  [[nodiscard]] const SensorReadings& sensors() const { return sensors_.readings(); }
  [[nodiscard]] const MultirotorModel& model() const { return model_; }

 private:
  void resolve_ground_contact();

  MultirotorModel model_;
  double step_size_;
  std::uint64_t seed_;
  std::uint64_t steps_ = 0;  // Counting steps avoids drift from summing step sizes.
  VehicleState state_;
  MotorCommands commands_{};
  Sensors sensors_;
};

}  // namespace sim
