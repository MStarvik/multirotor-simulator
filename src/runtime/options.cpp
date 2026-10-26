#include "runtime/options.hpp"

#include <charconv>
#include <cstdlib>
#include <format>
#include <print>
#include <stdexcept>
#include <string_view>
#include <system_error>

#include <unistd.h>

namespace sim::runtime {
namespace {

constexpr std::string_view usage = R"(Usage: {} [options]

Vehicle:
  --rotors N          number of rotors; even, 4 to 12 (default: 4)
  --arm-length M      distance from the center to each rotor in meters (default: 0.225)
  --mass KG           vehicle mass in kilograms (default: 1.0)

Flight controller:
  --sitl              wait for an external flight controller on the SITL socket
                      instead of flying with the built-in controller
  --socket-dir DIR    where to create sitl.sock and viz.sock
                      (default: $XDG_RUNTIME_DIR/multirotor-sim)

Simulation:
  --seed N            seed for sensor noise (default: 1)
  --speed FACTOR      speed relative to real time (default: 1)
  --realtime          headless: pace to the wall clock instead of running flat out
  --duration SECONDS  headless: simulated time to run (default: 10)

Viewer:
  --msaa N            anti-aliasing samples per pixel: 1 (off), 2, 4 or 8 (default: 4)

  --help              show this help
)";

// $XDG_RUNTIME_DIR is private to the user and cleaned up on logout, which is what
// sockets want. Fall back to a per-user directory under /tmp.
std::filesystem::path default_socket_directory() {
  if (const char* runtime_directory = std::getenv("XDG_RUNTIME_DIR");
      runtime_directory != nullptr && *runtime_directory != '\0') {
    return std::filesystem::path{runtime_directory} / "multirotor-sim";
  }
  return std::filesystem::temp_directory_path() / std::format("multirotor-sim-{}", getuid());
}

template <typename T>
T parse_number(std::string_view option, std::string_view text) {
  T value{};
  const char* const end = text.data() + text.size();
  const auto [parsed_end, error] = std::from_chars(text.data(), end, value);
  if (error != std::errc{} || parsed_end != end) {
    throw std::invalid_argument{std::format("invalid value for {}: '{}'", option, text)};
  }
  return value;
}

}  // namespace

std::optional<Options> parse_options(std::span<char* const> args) {
  Options options;
  const std::string_view program = args.empty() ? "multirotor-sim" : args.front();

  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string_view option = args[i];
    const auto value = [&] {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument{std::format("{} needs a value", option)};
      }
      return std::string_view{args[++i]};
    };

    if (option == "--help") {
      std::print(usage, program);
      return std::nullopt;
    } else if (option == "--rotors") {
      options.frame.rotor_count = parse_number<int>(option, value());
    } else if (option == "--arm-length") {
      options.frame.arm_length = parse_number<double>(option, value());
    } else if (option == "--mass") {
      options.frame.body.mass = parse_number<double>(option, value());
    } else if (option == "--sitl") {
      options.sitl = true;
    } else if (option == "--socket-dir") {
      options.socket_directory = value();
    } else if (option == "--seed") {
      options.seed = parse_number<std::uint64_t>(option, value());
    } else if (option == "--speed") {
      options.speed = parse_number<double>(option, value());
    } else if (option == "--realtime") {
      options.realtime = true;
    } else if (option == "--msaa") {
      options.msaa_samples = parse_number<std::uint32_t>(option, value());
    } else if (option == "--duration") {
      options.duration = parse_number<double>(option, value());
    } else {
      throw std::invalid_argument{std::format("unknown option '{}' (see --help)", option)};
    }
  }

  if (options.speed <= 0.0 || options.duration <= 0.0) {
    throw std::invalid_argument{"speed and duration must be positive"};
  }
  if (const auto n = options.msaa_samples; n != 1 && n != 2 && n != 4 && n != 8) {
    throw std::invalid_argument{"--msaa must be 1, 2, 4 or 8"};
  }
  if (options.socket_directory.empty()) {
    options.socket_directory = default_socket_directory();
  }
  return options;
}

}  // namespace sim::runtime
