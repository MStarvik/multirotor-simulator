#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "sim/multirotor.hpp"

namespace sim::runtime {

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
  bool realtime = false;   // Headless only: pace to the wall clock.
  double duration = 10.0;  // Headless and recorder: simulated seconds to run.
  std::filesystem::path output;  // Recorder only: the video file to write.
  std::uint32_t frame_rate = 30;  // Recorder only.
  std::uint32_t width = 1280;     // Recorder only: video size in pixels.
  std::uint32_t height = 720;
};

// Parses command-line arguments (including the program name). Returns nullopt
// after printing usage for --help; throws std::invalid_argument on bad input.
std::optional<Options> parse_options(std::span<char* const> args);

}  // namespace sim::runtime
