#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <stop_token>
#include <variant>
#include <vector>

#include "runtime/flight_controller.hpp"
#include "runtime/triple_buffer.hpp"
#include "sim/simulation.hpp"

namespace sim::runtime {

// What the simulation publishes after every step for observers such as the viewer.
struct Snapshot {
  double time = 0.0;  // Simulation time, s.
  VehicleState vehicle;
  SensorReadings sensors;
  double real_time_factor = 0.0;  // Recent simulated seconds per wall-clock second.
  bool paused = false;
  bool waiting_for_controller = false;  // The flight controller hasn't answered yet.
};

struct SetPilotInput {
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

// Runs a Simulation with a FlightController under a clock policy.
//
// run() executes on one thread (a dedicated one with a viewer, or the main thread
// when headless). One other thread may push() commands, which are applied between
// steps, and read the latest() snapshot, which is published after every step.
class SimRunner {
 public:
  SimRunner(Simulation simulation, std::unique_ptr<FlightController> controller, ClockPolicy clock);

  // Steps until the simulation time reaches `until` or a stop is requested.
  void run(std::stop_token stop, double until = std::numeric_limits<double>::infinity());

  void push(Command command);

  // The reference stays valid until the next call. Single consumer thread only.
  [[nodiscard]] const Snapshot& latest() { return snapshots_.latest(); }

 private:
  void apply_commands();
  // Returns false if the flight controller wasn't ready, so no step was taken.
  [[nodiscard]] bool step();
  void publish();

  Simulation simulation_;
  std::unique_ptr<FlightController> controller_;
  ClockPolicy clock_;
  bool paused_ = false;
  bool waiting_for_controller_ = false;
  double real_time_factor_ = 0.0;

  std::mutex commands_mutex_;
  std::vector<Command> pending_commands_;  // Guarded by commands_mutex_.
  std::vector<Command> applying_commands_;  // Reused between steps to avoid allocating.

  TripleBuffer<Snapshot> snapshots_;
};

}  // namespace sim::runtime
