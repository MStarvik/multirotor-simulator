#include <cstdio>
#include <cstdlib>
#include <exception>
#include <print>
#include <thread>
#include <utility>

#include "app/setup.hpp"
#include "ipc/socket_directory.hpp"
#include "ipc/sim_server.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"
#include "viewer/viewer.hpp"

int main(int argc, char** argv) {
  try {
    const auto options = sim::runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    if (!options) {
      return EXIT_SUCCESS;
    }

    const sim::ipc::SocketDirectory sockets{options->socket_directory};
    sim::runtime::SimRunner runner =
        sim::app::make_runner(*options, &sockets, sim::runtime::RealTime{.speed = options->speed});
    const sim::ipc::SimServer server{sockets.sim_socket(), runner, options->frame};
    // After the simulator socket, so that a flight controller that is already
    // waiting finds it as soon as its SITL socket accepts it.
    for (sim::runtime::VehicleSpec& vehicle : sim::app::initial_vehicles(*options)) {
      runner.add_vehicle(std::move(vehicle));
    }
    sim::viewer::Viewer viewer{runner, server,
                               {.follow = options->follow,
                                .msaa_samples = options->msaa_samples,
                                .camera_distance = options->camera_distance}};

    // Declared last, so it is stopped and joined before the runner and viewer go away.
    const std::jthread simulation_thread{[&runner](std::stop_token stop) {
      try {
        runner.run(stop);
      } catch (const std::exception& error) {
        std::println(stderr, "fatal: simulation failed: {}", error.what());
        std::abort();
      }
    }};
    viewer.run();
  } catch (const std::exception& error) {
    std::println(stderr, "fatal: {}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
