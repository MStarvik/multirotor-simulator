#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <numbers>
#include <print>
#include <string>
#include <utility>

#include "app/setup.hpp"
#include "control/demo_flight.hpp"
#include "ipc/sim_server.hpp"
#include "ipc/socket_directory.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

// Runs without any window or GPU, printing the state of each vehicle once per
// simulated second. With the built-in controller it flies a scripted flight (the script's
// stick inputs are passed on to an external flight controller too). Exits with
// failure if the state stops being finite.

namespace {

using namespace sim;
using namespace sim::runtime;

double degrees(double radians) { return radians * 180.0 / std::numbers::pi; }

// With several vehicles, each line starts with the vehicle's name, padded to
// `name_width`.
void print_state(double time, const VehicleSnapshot& vehicle, const std::string& name, std::size_t name_width) {
  const VehicleState& state = vehicle.state;
  const Euler attitude = to_euler(state.attitude);
  std::println(
      "{:{}}{:6.1f} s  N {:7.2f}  E {:7.2f}  alt {:6.2f} m  roll {:6.1f}°  pitch {:6.1f}°  yaw {:6.1f}°  "
      "baro {:9.1f} Pa",
      name_width > 0 ? name + "  " : "", name_width > 0 ? name_width + 2 : 0, time, state.position.x,
      state.position.y, 0.0 - state.position.z, degrees(attitude.roll), degrees(attitude.pitch),
      degrees(attitude.yaw), vehicle.sensors.barometer.pressure);
}

int run(const Options& options) {
  const ClockPolicy clock =
      options.realtime ? ClockPolicy{RealTime{.speed = options.speed}} : ClockPolicy{AsFastAsPossible{}};
  const ipc::SocketDirectory sockets{options.socket_directory};
  SimRunner runner = app::make_runner(options, &sockets, clock);
  const ipc::SimServer server{sockets.sim_socket(), runner, options.frame};
  // After the simulator socket, so that a flight controller that is already
  // waiting finds it as soon as its SITL socket accepts it.
  for (VehicleSpec& vehicle : app::initial_vehicles(options)) {
    runner.add_vehicle(std::move(vehicle));
  }

  const auto wall_start = std::chrono::steady_clock::now();
  control::DemoFlight demo_flight;
  for (double until = 1.0; ; until += 1.0) {
    const double end = std::min(until, options.duration);
    demo_flight.run_until(runner, end);

    const Snapshot& snapshot = runner.latest();
    const auto& vehicles = snapshot.roster->vehicles;
    std::size_t name_width = 0;
    if (vehicles.size() > 1) {
      name_width = std::ranges::max(vehicles, {}, [](const VehicleSpec& v) { return v.name.size(); }).name.size();
    }
    for (std::size_t i = 0; i < snapshot.vehicles.size(); ++i) {
      const VehicleState& state = snapshot.vehicles[i].state;
      const std::string& name = vehicles[i].name;
      print_state(snapshot.time, snapshot.vehicles[i], name, name_width);
      if (!is_finite(state.position) || !is_finite(state.attitude)) {
        std::println(stderr, "error: the simulation of vehicle {} diverged", name);
        return EXIT_FAILURE;
      }
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
