#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sim/multirotor.hpp"

namespace sim::runtime {

// How the viewer's camera follows a vehicle.
enum class CameraMode {
  follow,    // From a fixed direction, south-west of it and above.
  chase,     // From behind and above, turning with it.
  overhead,  // From straight above, north up.
  fixed,     // From a fixed position, turning to keep it in view.
};

struct Options {
  SymmetricFrame frame;  // Shared by all vehicles.
  // Vehicle names, which name their SITL sockets. Vehicle i starts i·spacing east of
  // the origin.
  std::vector<std::string> vehicles{"0"};
  double step_size = 0.001;  // s
  std::uint64_t seed = 1;    // Sensor noise seed.
  bool sitl = false;         // Wait for an external flight controller instead of the built-in one.
  std::string instance = "default";         // Names this simulator's socket directory.
  std::filesystem::path socket_directory;  // Where the SITL and drawing sockets are created.
  double speed = 1.0;      // Simulation speed relative to real time.
  std::uint32_t msaa_samples = 4;  // Viewer only: anti-aliasing samples per pixel.
  double camera_distance = 2.4;    // Viewer only: follow camera distance, m.
  std::string follow;              // Viewer and recorder: the vehicle the camera follows; empty for the first.
  CameraMode camera = CameraMode::follow;  // Viewer and recorder.
  Vec3 camera_position{-8.0, -8.0, -3.0};  // Viewer and recorder, CameraMode::fixed: world frame, m.
  bool no_viewer = false;            // Simulator: don't open a viewer.
  bool exit_with_simulator = false;  // Viewer: exit when the simulator does, rather than waiting for another.
  bool realtime = false;   // Headless only: pace to the wall clock.
  double duration = 10.0;  // Headless and recorder: simulated seconds to run.
  std::filesystem::path output;  // Recorder only: the video file to write.
  std::uint32_t frame_rate = 30;  // Recorder only.
  std::uint32_t width = 1280;     // Recorder only: video size in pixels.
  std::uint32_t height = 720;
};

// As --camera takes it.
[[nodiscard]] std::string_view to_string(CameraMode mode);

// Where the sockets of the simulator with this instance name are, by default.
[[nodiscard]] std::filesystem::path default_socket_directory(std::string_view instance);

// Parses command-line arguments (including the program name). Returns nullopt
// after printing usage for --help; throws std::invalid_argument on bad input.
std::optional<Options> parse_options(std::span<char* const> args);

}  // namespace sim::runtime
