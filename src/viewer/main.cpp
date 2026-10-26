#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <print>
#include <thread>
#include <utility>

#include "control/attitude_controller.hpp"
#include "ipc/sitl_controller.hpp"
#include "ipc/viz_server.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"
#include "sim/simulation.hpp"
#include "viewer/viewer.hpp"

int main(int argc, char** argv) {
  try {
    const auto options = sim::runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    if (!options) {
      return EXIT_SUCCESS;
    }

    const sim::MultirotorModel model = sim::make_multirotor(options->frame);
    // Before the SITL socket, so that a flight controller that is already waiting
    // finds the drawing socket as soon as the SITL socket accepts it.
    const sim::ipc::VizServer viz_server{options->socket_directory / "viz.sock"};
    std::unique_ptr<sim::runtime::FlightController> controller;
    if (options->sitl) {
      controller = std::make_unique<sim::ipc::SitlController>(model, options->step_size,
                                                              options->socket_directory / "sitl.sock");
    } else {
      controller = std::make_unique<sim::control::AttitudeController>(model);
    }
    sim::runtime::SimRunner runner{sim::Simulation{model, options->step_size, options->seed},
                                   std::move(controller), sim::runtime::RealTime{.speed = options->speed}};
    sim::viewer::Viewer viewer{runner, model, viz_server,
                               {.msaa_samples = options->msaa_samples,
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
