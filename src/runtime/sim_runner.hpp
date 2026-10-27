#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "runtime/flight_controller.hpp"
#include "runtime/triple_buffer.hpp"
#include "sim/simulation.hpp"

namespace sim::runtime {

enum class ControllerKind {
  built_in,  // The simulator's own attitude controller, flown with the sticks.
  sitl,      // An external flight controller on the vehicle's SITL socket.
};

// Everything needed to add a vehicle to the simulation.
struct VehicleSpec {
  std::string name;  // Unique in the simulation; names the vehicle's SITL socket.
  MultirotorModel model;
  VehicleState initial_state;  // Also where a reset puts it.
  ControllerKind controller = ControllerKind::built_in;
};

// The vehicles in the simulation. Replaced, never changed, when vehicles come or go,
// so snapshots share it.
struct Roster {
  std::uint64_t version = 0;  // Increases with every change.
  std::vector<VehicleSpec> vehicles;
};

struct VehicleSnapshot {
  VehicleState state;
  SensorReadings sensors;
  bool waiting_for_controller = false;  // Its flight controller hasn't answered yet.
};

// What the simulation publishes after every step for observers such as the viewer.
struct Snapshot {
  double time = 0.0;  // Simulation time, s.
  std::shared_ptr<const Roster> roster;
  std::vector<VehicleSnapshot> vehicles;  // In roster order.
  double real_time_factor = 0.0;  // Recent simulated seconds per wall-clock second.
  bool paused = false;
};

struct SetPilotInput {
  std::string vehicle;  // Empty for every vehicle.
  PilotInput input;
};
struct Reset {};
struct SetPaused {
  bool paused;
};
// Only for a RealTime clock.
struct SetSpeed {
  double speed;
};
using Command = std::variant<SetPilotInput, Reset, SetPaused, SetSpeed>;

enum class Status { ok, invalid, not_found, exists, unsupported, failed };

struct CommandResult {
  Status status = Status::ok;
  std::string message;  // Explains a failure.
};

// Called on the simulation thread once a command has been applied, or has failed.
using Completion = std::function<void(const CommandResult&)>;

// Steps run in step with wall-clock time, scaled by speed (2 = twice as fast).
struct RealTime {
  double speed = 1.0;
};
// Steps run back to back, as fast as the CPU allows.
struct AsFastAsPossible {};
using ClockPolicy = std::variant<RealTime, AsFastAsPossible>;

// Makes the flight controller for a new vehicle. Throws if it can't.
using ControllerFactory = std::function<std::unique_ptr<FlightController>(const VehicleSpec&)>;

// Runs a Simulation with a FlightController per vehicle under a clock policy. A
// step is taken once every flight controller has answered it, so an external one
// that is slow or not yet connected holds the whole simulation.
//
// run() executes on one thread (a dedicated one with a viewer, or the main thread
// when headless). Other threads may push() commands, which are applied between
// steps. One thread may read the latest() snapshot, which is published after every
// step.
class SimRunner {
 public:
  SimRunner(double step_size, std::uint64_t seed, ControllerFactory make_controller, ClockPolicy clock);

  // Only before run() is first called. Throws if the vehicle can't be added.
  void add_vehicle(VehicleSpec spec);

  // Steps until the simulation time reaches `until` or a stop is requested.
  void run(std::stop_token stop, double until = std::numeric_limits<double>::infinity());

  // Thread safe. `done` is called on the simulation thread.
  void push(Command command, Completion done = {});

  // The reference stays valid until the next call. Single consumer thread only.
  [[nodiscard]] const Snapshot& latest() { return snapshots_.latest(); }

 private:
  struct Vehicle {
    std::unique_ptr<FlightController> controller;
    // The controller's answer for the current step, kept until all have answered.
    std::optional<MotorCommands> answer;
  };

  void apply_commands();
  [[nodiscard]] CommandResult apply(const Command& command);
  // Returns the vehicle's index in the roster, if there is one with that name.
  [[nodiscard]] std::optional<std::size_t> find_vehicle(const std::string& name) const;
  // Returns false if some flight controller wasn't ready, so no step was taken.
  [[nodiscard]] bool step();
  void publish();

  Simulation simulation_;
  ControllerFactory make_controller_;
  std::shared_ptr<const Roster> roster_;
  std::vector<Vehicle> vehicles_;  // In roster order, which is also the simulation's.
  ClockPolicy clock_;
  bool paused_ = false;
  bool waiting_for_controllers_ = false;
  bool repace_ = false;  // Restart real-time pacing from the current time.
  double real_time_factor_ = 0.0;

  std::mutex commands_mutex_;
  // Guarded by commands_mutex_.
  std::vector<std::pair<Command, Completion>> pending_commands_;
  // Reused between steps to avoid allocating.
  std::vector<std::pair<Command, Completion>> applying_commands_;

  Snapshot next_snapshot_;  // Reused between steps to avoid allocating.
  TripleBuffer<Snapshot> snapshots_;
};

}  // namespace sim::runtime
