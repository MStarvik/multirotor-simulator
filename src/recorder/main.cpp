#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <print>
#include <stdexcept>
#include <stop_token>

#include "app/setup.hpp"
#include "control/demo_flight.hpp"
#include "ipc/socket_directory.hpp"
#include "ipc/viz_server.hpp"
#include "recorder/video_encoder.hpp"
#include "render/offscreen_renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "scene/scene_view.hpp"
#include "sim/multirotor.hpp"

// Records a flight to a video file, rendered offscreen: no window, compositor or
// display needed. Frames are taken at fixed intervals of simulated time, so the
// video plays back in real time however long it took to make. Flies the demo flight
// with the built-in controller, or whatever an external one does with --sitl, and
// shows drawings from the viz socket like the viewer.

namespace {

using namespace sim;

int record(const runtime::Options& options) {
  if (options.output.empty()) {
    throw std::invalid_argument{"--output is required (see --help)"};
  }

  const MultirotorModel model = make_multirotor(options.frame);
  const ipc::SocketDirectory sockets{options.socket_directory};
  // Before the SITL sockets, so that a flight controller that is already waiting
  // finds the drawing socket as soon as its SITL socket accepts it.
  const ipc::VizServer viz_server{sockets.viz_socket()};
  runtime::SimRunner runner = app::make_runner(options, model, &sockets, runtime::AsFastAsPossible{});

  scene::SceneView scene{model, options.camera_distance};
  const render::VulkanContext vulkan;
  render::OffscreenRenderer renderer{vulkan, {.width = options.width, .height = options.height},
                                     options.msaa_samples, scene.meshes()};
  recorder::VideoEncoder encoder{options.output, options.width, options.height, options.frame_rate};

  const auto frame_count = static_cast<std::uint64_t>(std::llround(options.duration * options.frame_rate));
  const auto wall_start = std::chrono::steady_clock::now();
  control::DemoFlight demo_flight;
  for (std::uint64_t frame = 0; frame < frame_count; ++frame) {
    const double time = static_cast<double>(frame) / options.frame_rate;
    if (options.sitl) {
      runner.run(std::stop_token{}, time);
    } else {
      demo_flight.run_until(runner, time);
    }
    scene.update_drawings(viz_server);
    const runtime::Snapshot& snapshot = runner.latest();
    encoder.write(renderer.render(scene.frame(snapshot, options.follow, options.width, options.height)));
  }
  encoder.finish();

  const std::chrono::duration<double> wall_time = std::chrono::steady_clock::now() - wall_start;
  std::println("Wrote {} frames ({:.1f} s at {} fps, {}×{}, {}× MSAA) to {} in {:.1f} s", frame_count,
               options.duration, options.frame_rate, options.width, options.height, renderer.msaa_samples(),
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
