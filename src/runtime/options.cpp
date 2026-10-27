#include "runtime/options.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdlib>
#include <format>
#include <iterator>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <unistd.h>

#include "runtime/sim_runner.hpp"

namespace sim::runtime {
namespace {

constexpr std::string_view usage = R"(Usage: {} [options]

Vehicles:
  --vehicle NAME      add a vehicle; repeat for more (default: one vehicle named 0)
  --vehicles N        add N vehicles named 0 to N-1
  --rotors N          number of rotors; even, 4 to 12 (default: 4)
  --arm-length M      distance from the center to each rotor in meters (default: 0.225)
  --mass KG           vehicle mass in kilograms (default: 1.0)

Flight controller:
  --sitl              wait for an external flight controller for each vehicle, on
                      its socket vehicles/NAME.sock, instead of flying with the
                      built-in controller
  --instance NAME     names this simulator, so that several can run at once: its
                      sockets go in $XDG_RUNTIME_DIR/multirotor-sim/NAME
                      (default: default)
  --socket-dir DIR    put the sockets in DIR instead

Simulation:
  --seed N            seed for sensor noise (default: 1)
  --speed FACTOR      speed relative to real time (default: 1)
  --realtime          headless: pace to the wall clock instead of running unpaced
  --duration SECONDS  headless and recorder: simulated time to run (default: 10)

Viewer and recorder:
  --msaa N            anti-aliasing samples per pixel: 1 (off), 2, 4 or 8 (default: 4)
  --follow NAME       the vehicle the camera follows (default: the first)
  --camera MODE       how it follows: follow (from the south-west, above), chase
                      (from behind), overhead, or fixed (from --camera-position)
                      (default: follow)
  --camera-distance M distance from the camera to the vehicle (default: 2.4)
  --camera-position N,E,D  where the fixed camera is, m (default: -8,-8,-3)
  --no-viewer         simulator: don't open a viewer; attach one later with
                      multirotor-sim-viewer
  --exit-with-simulator  viewer: exit when the simulator does, instead of
                      waiting for the next one

Recorder:
  --output FILE       video to write (required): .webm for a VP9 video, or .avif
                      for an animated image that browsers autoplay
  --size WxH          video size in pixels (default: 1280x720)
  --fps N             frames per second of simulated time (default: 30)

  --help              show this help
)";

// Instance and vehicle names become part of socket paths, which are short.
std::string_view validated_name(std::string_view option, std::string_view name) {
  if (!is_valid_name(name)) {
    throw std::invalid_argument{
        std::format("invalid name for {}: '{}'; use up to {} letters, digits, '-', '_' and '.', "
                    "not starting with '.'",
                    option, name, max_name_length)};
  }
  return name;
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

CameraMode parse_camera_mode(std::string_view text) {
  if (text == "follow") {
    return CameraMode::follow;
  }
  if (text == "chase") {
    return CameraMode::chase;
  }
  if (text == "overhead") {
    return CameraMode::overhead;
  }
  if (text == "fixed") {
    return CameraMode::fixed;
  }
  throw std::invalid_argument{std::format("--camera must be follow, chase, overhead or fixed, not '{}'", text)};
}

// N,E,D in meters.
Vec3 parse_position(std::string_view option, std::string_view text) {
  std::array<double, 3> coordinates{};
  std::size_t start = 0;
  for (std::size_t i = 0; i < coordinates.size(); ++i) {
    const std::size_t comma = i + 1 < coordinates.size() ? text.find(',', start) : text.size();
    if (comma == std::string_view::npos) {
      throw std::invalid_argument{std::format("{} takes three coordinates, N,E,D", option)};
    }
    coordinates[i] = parse_number<double>(option, text.substr(start, comma - start));
    start = comma + 1;
  }
  return {coordinates[0], coordinates[1], coordinates[2]};
}

}  // namespace

std::string_view to_string(CameraMode mode) {
  switch (mode) {
    case CameraMode::follow: return "follow";
    case CameraMode::chase: return "chase";
    case CameraMode::overhead: return "overhead";
    case CameraMode::fixed: return "fixed";
  }
  return "follow";
}

// $XDG_RUNTIME_DIR is private to the user and cleaned up on logout, which is what
// sockets want. Fall back to a per-user directory under /tmp.
std::filesystem::path default_socket_directory(std::string_view instance) {
  if (const char* runtime_directory = std::getenv("XDG_RUNTIME_DIR");
      runtime_directory != nullptr && *runtime_directory != '\0') {
    return std::filesystem::path{runtime_directory} / "multirotor-sim" / instance;
  }
  return std::filesystem::temp_directory_path() / std::format("multirotor-sim-{}", getuid()) / instance;
}

std::optional<Options> parse_options(std::span<char* const> args) {
  Options options;
  options.vehicles.clear();
  bool instance_given = false;
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
    } else if (option == "--vehicle") {
      options.vehicles.emplace_back(validated_name(option, value()));
    } else if (option == "--vehicles") {
      const auto count = parse_number<int>(option, value());
      if (count < 1 || count > 64) {
        throw std::invalid_argument{"--vehicles must be between 1 and 64"};
      }
      for (int n = 0; n < count; ++n) {
        options.vehicles.push_back(std::to_string(n));
      }
    } else if (option == "--rotors") {
      options.frame.rotor_count = parse_number<int>(option, value());
    } else if (option == "--arm-length") {
      options.frame.arm_length = parse_number<double>(option, value());
    } else if (option == "--mass") {
      options.frame.body.mass = parse_number<double>(option, value());
    } else if (option == "--sitl") {
      options.sitl = true;
    } else if (option == "--instance") {
      options.instance = validated_name(option, value());
      instance_given = true;
    } else if (option == "--socket-dir") {
      options.socket_directory = value();
    } else if (option == "--seed") {
      options.seed = parse_number<std::uint64_t>(option, value());
    } else if (option == "--speed") {
      options.speed = parse_number<double>(option, value());
    } else if (option == "--realtime") {
      options.realtime = true;
    } else if (option == "--camera-distance") {
      options.camera_distance = parse_number<double>(option, value());
    } else if (option == "--follow") {
      options.follow = value();
    } else if (option == "--camera") {
      options.camera = parse_camera_mode(value());
    } else if (option == "--camera-position") {
      options.camera_position = parse_position(option, value());
    } else if (option == "--no-viewer") {
      options.no_viewer = true;
    } else if (option == "--exit-with-simulator") {
      options.exit_with_simulator = true;
    } else if (option == "--output") {
      options.output = value();
    } else if (option == "--fps") {
      options.frame_rate = parse_number<std::uint32_t>(option, value());
    } else if (option == "--size") {
      const std::string_view size = value();
      const auto separator = size.find('x');
      if (separator == std::string_view::npos) {
        throw std::invalid_argument{std::format("--size must look like 1280x720, got '{}'", size)};
      }
      options.width = parse_number<std::uint32_t>(option, size.substr(0, separator));
      options.height = parse_number<std::uint32_t>(option, size.substr(separator + 1));
    } else if (option == "--msaa") {
      options.msaa_samples = parse_number<std::uint32_t>(option, value());
    } else if (option == "--duration") {
      options.duration = parse_number<double>(option, value());
    } else {
      throw std::invalid_argument{std::format("unknown option '{}' (see --help)", option)};
    }
  }

  if (options.speed <= 0.0 || options.duration <= 0.0 || options.camera_distance <= 0.0) {
    throw std::invalid_argument{"speed, duration and camera distance must be positive"};
  }
  if (options.frame_rate == 0 || options.width == 0 || options.height == 0) {
    throw std::invalid_argument{"frame rate and video size must be positive"};
  }
  if (const auto n = options.msaa_samples; n != 1 && n != 2 && n != 4 && n != 8) {
    throw std::invalid_argument{"--msaa must be 1, 2, 4 or 8"};
  }

  if (options.vehicles.empty()) {
    options.vehicles.emplace_back("0");
  }
  for (auto i = options.vehicles.begin(); i != options.vehicles.end(); ++i) {
    if (std::find(std::next(i), options.vehicles.end(), *i) != options.vehicles.end()) {
      throw std::invalid_argument{std::format("there are two vehicles named '{}'", *i)};
    }
  }
  if (!options.follow.empty() && std::ranges::find(options.vehicles, options.follow) == options.vehicles.end()) {
    throw std::invalid_argument{std::format("--follow: there is no vehicle named '{}'", options.follow)};
  }

  if (instance_given && !options.socket_directory.empty()) {
    throw std::invalid_argument{"give either --instance or --socket-dir, not both"};
  }
  if (options.socket_directory.empty()) {
    options.socket_directory = default_socket_directory(options.instance);
  }
  return options;
}

}  // namespace sim::runtime
