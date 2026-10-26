#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <numbers>
#include <print>
#include <stop_token>
#include <utility>

#include "control/attitude_controller.hpp"
#include "ipc/sitl_controller.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"
#include "sim/simulation.hpp"

// Runs without any window or GPU, printing the vehicle state once per simulated
// second. With the built-in controller it flies a scripted flight (the script's
// stick inputs are passed on to an external flight controller too). Exits with
// failure if the state stops being finite.

namespace {

using namespace sim;
using namespace sim::runtime;

struct ScriptStep {
  double start;  // Simulation time, s.
  PilotInput input;
};

// Take off, hover, fly forward, then hover again.
constexpr std::array script{
    ScriptStep{.start = 0.0, .input = {.throttle = 0.5}},
    ScriptStep{.start = 3.0, .input = {}},
    ScriptStep{.start = 5.0, .input = {.pitch = 0.3}},
    ScriptStep{.start = 7.0, .input = {}},
};

double degrees(double radians) { return radians * 180.0 / std::numbers::pi; }

void print_state(const Snapshot& snapshot) {
  const VehicleState& vehicle = snapshot.vehicle;
  const Euler attitude = to_euler(vehicle.attitude);
  std::println(
      "{:6.1f} s  N {:7.2f}  E {:7.2f}  alt {:6.2f} m  roll {:6.1f}°  pitch {:6.1f}°  yaw {:6.1f}°  "
      "baro {:9.1f} Pa",
      snapshot.time, vehicle.position.x, vehicle.position.y, 0.0 - vehicle.position.z,
      degrees(attitude.roll), degrees(attitude.pitch), degrees(attitude.yaw),
      snapshot.sensors.barometer.pressure);
}

int run(const Options& options) {
  const MultirotorModel model = make_multirotor(options.frame);
  const ClockPolicy clock =
      options.realtime ? ClockPolicy{RealTime{.speed = options.speed}} : ClockPolicy{AsFastAsPossible{}};
  std::unique_ptr<FlightController> controller;
  if (options.sitl) {
    controller = std::make_unique<ipc::SitlController>(model, options.step_size,
                                                       options.socket_directory / "sitl.sock");
  } else {
    controller = std::make_unique<control::AttitudeController>(model);
  }
  SimRunner runner{Simulation{model, options.step_size, options.seed}, std::move(controller), clock};

  const auto wall_start = std::chrono::steady_clock::now();
  std::size_t next_step = 0;
  for (double until = 1.0; ; until += 1.0) {
    const double end = std::min(until, options.duration);
    while (next_step < script.size() && script[next_step].start < end) {
      runner.push(SetPilotInput{script[next_step++].input});
    }
    runner.run(std::stop_token{}, end);

    const Snapshot& snapshot = runner.latest();
    print_state(snapshot);
    if (!is_finite(snapshot.vehicle.position) || !is_finite(snapshot.vehicle.attitude)) {
      std::println(stderr, "error: the simulation diverged");
      return EXIT_FAILURE;
    }
    if (end >= options.duration) {
      break;
    }
  }

  const std::chrono::duration<double> wall_time = std::chrono::steady_clock::now() - wall_start;
  std::println("Simulated {:.1f} s in {:.3f} s of wall time ({:.0f}× real time)", options.duration,
               wall_time.count(), options.duration / wall_time.count());
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_options({argv, static_cast<std::size_t>(argc)});
    return options ? run(*options) : EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::println(stderr, "error: {}", error.what());
    return EXIT_FAILURE;
  }
}
