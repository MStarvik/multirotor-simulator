#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>

#include "sim/multirotor.hpp"

namespace sim::runtime {

struct Options {
  SymmetricFrame frame;
  double step_size = 0.001;  // s
  std::uint64_t seed = 1;    // Sensor noise seed.
  bool sitl = false;         // Wait for an external flight controller instead of the built-in one.
  std::filesystem::path socket_directory;  // Where the SITL and drawing sockets are created.
  double speed = 1.0;      // Simulation speed relative to real time.
  std::uint32_t msaa_samples = 4;  // Viewer only: anti-aliasing samples per pixel.
  bool realtime = false;   // Headless only: pace to the wall clock.
  double duration = 10.0;  // Headless only: simulated seconds to run.
};

// Parses command-line arguments (including the program name). Returns nullopt
// after printing usage for --help; throws std::invalid_argument on bad input.
std::optional<Options> parse_options(std::span<char* const> args);

}  // namespace sim::runtime
