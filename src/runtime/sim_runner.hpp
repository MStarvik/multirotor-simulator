#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <variant>
#include <vector>

#include "runtime/flight_controller.hpp"
#include "runtime/triple_buffer.hpp"
#include "sim/simulation.hpp"

namespace sim::runtime {

struct VehicleSnapshot {
  VehicleState state;
  SensorReadings sensors;
  bool waiting_for_controller = false;  // Its flight controller hasn't answered yet.
};

// What the simulation publishes after every step for observers such as the viewer.
struct Snapshot {
  double time = 0.0;  // Simulation time, s.
  std::vector<VehicleSnapshot> vehicles;  // In the simulation's order.
  double real_time_factor = 0.0;  // Recent simulated seconds per wall-clock second.
  bool paused = false;
};

struct SetPilotInput {
  std::size_t vehicle;
  PilotInput input;
};
struct Reset {};
struct SetPaused {
  bool paused;
};
using Command = std::variant<SetPilotInput, Reset, SetPaused>;

// Steps run in step with wall-clock time, scaled by speed (2 = twice as fast).
struct RealTime {
  double speed = 1.0;
};
// Steps run back to back, as fast as the CPU allows.
struct AsFastAsPossible {};
using ClockPolicy = std::variant<RealTime, AsFastAsPossible>;

// Runs a Simulation with a FlightController per vehicle under a clock policy. A
// step is taken once every flight controller has answered it, so an external one
// that is slow or not yet connected holds the whole simulation.
//
// run() executes on one thread (a dedicated one with a viewer, or the main thread
// when headless). One other thread may push() commands, which are applied between
// steps, and read the latest() snapshot, which is published after every step.
class SimRunner {
 public:
  // One controller per vehicle, in the simulation's order.
  SimRunner(Simulation simulation, std::vector<std::unique_ptr<FlightController>> controllers,
            ClockPolicy clock);

  // Steps until the simulation time reaches `until` or a stop is requested.
  void run(std::stop_token stop, double until = std::numeric_limits<double>::infinity());

  void push(Command command);

  // Safe to call from any thread.
  [[nodiscard]] std::size_t vehicle_count() const { return controllers_.size(); }

  // The reference stays valid until the next call. Single consumer thread only.
  [[nodiscard]] const Snapshot& latest() { return snapshots_.latest(); }

 private:
  void apply_commands();
  // Returns false if some flight controller wasn't ready, so no step was taken.
  [[nodiscard]] bool step();
  void publish();

  Simulation simulation_;
  std::vector<std::unique_ptr<FlightController>> controllers_;
  // Each controller's answer for the current step, kept until all have answered.
  std::vector<std::optional<MotorCommands>> answers_;
  ClockPolicy clock_;
  bool paused_ = false;
  bool waiting_for_controllers_ = false;
  double real_time_factor_ = 0.0;

  std::mutex commands_mutex_;
  std::vector<Command> pending_commands_;  // Guarded by commands_mutex_.
  std::vector<Command> applying_commands_;  // Reused between steps to avoid allocating.

  Snapshot next_snapshot_;  // Reused between steps to avoid allocating.
  TripleBuffer<Snapshot> snapshots_;
};

}  // namespace sim::runtime
