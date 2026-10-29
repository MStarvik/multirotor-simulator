#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <print>
#include <stdexcept>
#include <stop_token>
#include <utility>

#include "app/setup.hpp"
#include "ipc/socket_directory.hpp"
#include "ipc/sim_server.hpp"
#include "recorder/video_encoder.hpp"
#include "render/offscreen_renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "scene/scene_view.hpp"
#include "sim/multirotor.hpp"

// Records a flight to a video file, rendered offscreen: no window, compositor or
// display needed. Frames are taken at fixed intervals of simulated time, so the
// video plays back in real time however long it took to make. Like the simulator,
// every vehicle waits for an external flight controller on its SITL socket, such as
// multirotor-sim-pilot --demo. Shows drawings from the simulator socket like the
// viewer.

namespace {

using namespace sim;

int record(const runtime::Options& options) {
  if (options.output.empty()) {
    throw std::invalid_argument{"--output is required (see --help)"};
  }

  const ipc::SocketDirectory sockets{options.socket_directory};
  runtime::SimRunner runner = app::make_runner(options, sockets, runtime::AsFastAsPossible{});
  const ipc::SimServer server{sockets.sim_socket(), runner, options.frame};
  // After the simulator socket, so that a flight controller that is already
  // waiting finds it as soon as its SITL socket accepts it.
  for (runtime::VehicleSpec& vehicle : app::initial_vehicles(options)) {
    runner.add_vehicle(std::move(vehicle));
  }

  scene::SceneView scene{
      {.mode = options.camera, .distance = options.camera_distance, .position = options.camera_position}};
  const render::VulkanContext vulkan;
  render::OffscreenRenderer renderer{vulkan, {.width = options.width, .height = options.height},
                                     options.msaa_samples, scene.meshes()};
  recorder::VideoEncoder encoder{options.output, options.width, options.height, options.frame_rate};

  const double duration = options.duration.value_or(10.0);
  const auto frame_count = static_cast<std::uint64_t>(std::llround(duration * options.frame_rate));
  const auto wall_start = std::chrono::steady_clock::now();
  for (std::uint64_t frame = 0; frame < frame_count; ++frame) {
    runner.run(std::stop_token{}, static_cast<double>(frame) / options.frame_rate);
    scene.update_drawings(server.drawings());
    const runtime::Snapshot& snapshot = runner.latest();
    if (scene.update_meshes(snapshot)) {
      renderer.set_meshes(scene.meshes());
    }
    const auto& vehicles = snapshot.roster->vehicles;
    const auto follow = static_cast<std::size_t>(
        std::ranges::find(vehicles, options.follow, &runtime::VehicleSpec::name) - vehicles.begin());
    encoder.write(renderer.render(scene.frame(snapshot, follow < vehicles.size() ? follow : 0, options.width,
                                              options.height)));
  }
  encoder.finish();

  const std::chrono::duration<double> wall_time = std::chrono::steady_clock::now() - wall_start;
  std::println("Wrote {} frames ({:.1f} s at {} fps, {}×{}, {}× MSAA) to {} in {:.1f} s", frame_count,
               duration, options.frame_rate, options.width, options.height, renderer.msaa_samples(),
               options.output.native(), wall_time.count());
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  // If ffmpeg exits early, writing to it should fail with EPIPE, not kill us.
  std::signal(SIGPIPE, SIG_IGN);
  try {
    const auto options = runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    return options ? record(*options) : EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::println(stderr, "error: {}", error.what());
    return EXIT_FAILURE;
  }
}
