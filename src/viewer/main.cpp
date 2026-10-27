#include <cstdio>
#include <cstdlib>
#include <exception>
#include <print>

#include "runtime/options.hpp"
#include "viewer/viewer.hpp"

// Shows a running simulator in a window. Any number of viewers can watch one
// simulator, each with its own camera; one started before the simulator waits for
// it, and one whose simulator exits waits for the next (unless told to exit too).

int main(int argc, char** argv) {
  try {
    const auto options = sim::runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    if (!options) {
      return EXIT_SUCCESS;
    }
    sim::viewer::Viewer viewer{{
        .socket = options->socket_directory / "sim.sock",
        .follow = options->follow,
        .camera = {.mode = options->camera,
                   .distance = options->camera_distance,
                   .position = options->camera_position},
        .msaa_samples = options->msaa_samples,
        .exit_with_simulator = options->exit_with_simulator,
    }};
    viewer.run();
  } catch (const std::exception& error) {
    std::println(stderr, "fatal: {}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
