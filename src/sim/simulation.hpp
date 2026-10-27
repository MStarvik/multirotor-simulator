#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "sim/math.hpp"
#include "sim/multirotor.hpp"
#include "sim/sensors.hpp"
#include "sim/vehicle_state.hpp"

namespace sim {

// Normalized speed command per rotor, in [0, 1].
using MotorCommands = std::array<double, max_rotors>;

// One multirotor in a Simulation: its rigid-body dynamics, motors and sensors.
// Vehicles don't interact with each other, only with the ground.
class Vehicle {
 public:
  [[nodiscard]] const VehicleState& state() const { return state_; }
  // Measurements of the current state.
  [[nodiscard]] const SensorReadings& sensors() const { return sensors_.readings(); }
  [[nodiscard]] const MultirotorModel& model() const { return model_; }

 private:
  friend class Simulation;

  Vehicle(MultirotorModel model, const VehicleState& initial_state, double step_size, std::uint64_t seed);

  void step(std::uint64_t step);  // The number of the step this one completes.
  void reset();
  void resolve_ground_contact();

  MultirotorModel model_;
  VehicleState initial_state_;
  double step_size_;
  std::uint64_t seed_;
  VehicleState state_;
  MotorCommands commands_{};
  Sensors sensors_;
};

// Any number of vehicles, advanced together in fixed steps.
//
// This is the whole simulation: it knows nothing about wall-clock time, threads,
// or rendering, so it runs identically headless, in a viewer, or in tests.
class Simulation {
 public:
  // The seed drives all sensor noise, so equal seeds and inputs give equal runs.
  // Each vehicle gets its own noise, derived from the seed and how many vehicles
  // were added before it.
  explicit Simulation(double step_size = 0.001, std::uint64_t seed = 1);

  // Returns the new vehicle's index. Vehicles are numbered in the order they are
  // added; removing one moves those after it down by one.
  std::size_t add_vehicle(MultirotorModel model, const VehicleState& initial_state = {});
  void remove_vehicle(std::size_t vehicle);

  // Commands applied from the next step on, until replaced.
  void set_motor_commands(std::size_t vehicle, const MotorCommands& commands);

  void step();
  // Puts every vehicle back in the state it was added with.
  void reset();

  [[nodiscard]] std::uint64_t step_count() const { return steps_; }
  [[nodiscard]] double time() const { return static_cast<double>(steps_) * step_size_; }
  [[nodiscard]] double step_size() const { return step_size_; }
  [[nodiscard]] std::span<const Vehicle> vehicles() const { return vehicles_; }

 private:
  double step_size_;
  std::uint64_t seed_;
  std::uint64_t steps_ = 0;  // Counting steps avoids drift from summing step sizes.
  std::uint64_t vehicles_added_ = 0;
  std::vector<Vehicle> vehicles_;
};

}  // namespace sim
