#include <cstdio>
#include <cstdlib>
#include <exception>
#include <print>
#include <thread>

#include "app/setup.hpp"
#include "ipc/socket_directory.hpp"
#include "ipc/viz_server.hpp"
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

    const sim::MultirotorModel model = sim::make_multirotor(options->frame);
    const sim::ipc::SocketDirectory sockets{options->socket_directory};
    // Before the SITL sockets, so that a flight controller that is already waiting
    // finds the drawing socket as soon as its SITL socket accepts it.
    const sim::ipc::VizServer viz_server{sockets.viz_socket()};
    sim::runtime::SimRunner runner =
        sim::app::make_runner(*options, model, &sockets, sim::runtime::RealTime{.speed = options->speed});
    sim::viewer::Viewer viewer{runner, model, viz_server,
                               {.vehicle_names = options->vehicles,
                                .follow = options->follow,
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
