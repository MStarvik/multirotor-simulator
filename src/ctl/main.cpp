#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

#include <poll.h>
#include <sys/socket.h>

#include <multirotor_sim/sim.h>

#include "ipc/unix_socket.hpp"
#include "runtime/options.hpp"

// Sends one request to a running simulator over its simulator socket, and reports
// the answer. Exits with failure if the simulator refuses it.

namespace {

using sim::ipc::UniqueFd;

constexpr std::string_view usage = R"(Usage: {} [--instance NAME | --socket-dir DIR] COMMAND

Commands:
  pause                 pause the simulation
  resume                resume it
  reset                 put every vehicle back where it started
  speed FACTOR          run at FACTOR times real time
  spawn NAME [options]  add a vehicle, with the simulator's frame unless given:
      --sitl              wait for an external flight controller on its socket
                          vehicles/NAME.sock, instead of the built-in controller
      --at N,E[,D]        where it starts, m (default: 0,0,0; D is down, so <= 0)
      --yaw DEGREES       heading, clockwise from north (default: 0)
      --rotors N, --arm-length M, --mass KG
  remove NAME           remove a vehicle

The simulator is the one named by --instance (default: default), or the one with
its sockets in --socket-dir.
)";

constexpr int reply_timeout_ms = 5000;

double parse_number(std::string_view what, std::string_view text) {
  double value = 0.0;
  const char* const end = text.data() + text.size();
  const auto [parsed_end, error] = std::from_chars(text.data(), end, value);
  if (error != std::errc{} || parsed_end != end) {
    throw std::invalid_argument{std::format("invalid value for {}: '{}'", what, text)};
  }
  return value;
}

void copy_name(std::string_view name, char (&to)[SIM_NAME_SIZE]) {
  if (name.empty() || name.size() > SIM_NAME_SIZE) {
    throw std::invalid_argument{std::format("vehicle names are 1 to {} characters", SIM_NAME_SIZE)};
  }
  std::memcpy(to, name.data(), name.size());
}

sim_header header(std::uint32_t type) {
  return {.type = type, .version = SIM_PROTOCOL_VERSION, .id = 1, .reserved = 0};
}

// The spawn command's arguments after the name.
sim_ctl_spawn_vehicle parse_spawn(std::string_view name, std::span<char* const> args) {
  sim_ctl_spawn_vehicle request{};
  request.header = header(SIM_CTL_SPAWN_VEHICLE);
  copy_name(name, request.name);
  request.controller = SIM_CONTROLLER_BUILT_IN;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string_view option = args[i];
    const auto value = [&] {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument{std::format("{} needs a value", option)};
      }
      return std::string_view{args[++i]};
    };
    if (option == "--sitl") {
      request.controller = SIM_CONTROLLER_SITL;
    } else if (option == "--at") {
      const std::string_view text = value();
      std::size_t axis = 0;
      for (std::size_t start = 0; start <= text.size(); ++axis) {
        const std::size_t comma = std::min(text.find(',', start), text.size());
        if (axis >= 3) {
          throw std::invalid_argument{"--at takes two or three coordinates"};
        }
        request.position[axis] = parse_number(option, text.substr(start, comma - start));
        start = comma + 1;
      }
      if (axis < 2) {
        throw std::invalid_argument{"--at takes two or three coordinates"};
      }
    } else if (option == "--yaw") {
      request.yaw = parse_number(option, value()) * std::numbers::pi / 180.0;
    } else if (option == "--rotors") {
      request.rotor_count = static_cast<std::uint32_t>(parse_number(option, value()));
    } else if (option == "--arm-length") {
      request.arm_length = parse_number(option, value());
    } else if (option == "--mass") {
      request.mass = parse_number(option, value());
    } else {
      throw std::invalid_argument{std::format("unknown spawn option '{}'", option)};
    }
  }
  return request;
}

// The request for a command, as bytes to send.
std::vector<std::byte> make_request(std::string_view command, std::span<char* const> args) {
  const auto bytes = [](const auto& message) {
    const auto view = std::as_bytes(std::span{&message, 1});
    return std::vector<std::byte>(view.begin(), view.end());
  };
  const auto argument = [&](std::size_t i) {
    if (i >= args.size()) {
      throw std::invalid_argument{std::format("{} needs more arguments (see --help)", command)};
    }
    return std::string_view{args[i]};
  };
  const auto no_more = [&](std::size_t count) {
    if (args.size() > count) {
      throw std::invalid_argument{std::format("too many arguments for {}", command)};
    }
  };

  if (command == "pause" || command == "resume") {
    no_more(0);
    return bytes(sim_ctl_pause{.header = header(SIM_CTL_PAUSE), .paused = command == "pause", .reserved = 0});
  }
  if (command == "reset") {
    no_more(0);
    return bytes(header(SIM_CTL_RESET));
  }
  if (command == "speed") {
    no_more(1);
    return bytes(sim_ctl_set_speed{.header = header(SIM_CTL_SET_SPEED), .speed = parse_number("speed", argument(0))});
  }
  if (command == "spawn") {
    return bytes(parse_spawn(argument(0), args.subspan(1)));
  }
  if (command == "remove") {
    no_more(1);
    sim_ctl_remove_vehicle request{.header = header(SIM_CTL_REMOVE_VEHICLE), .name = {}};
    copy_name(argument(0), request.name);
    return bytes(request);
  }
  throw std::invalid_argument{std::format("unknown command '{}' (see --help)", command)};
}

int run(std::span<char* const> args) {
  const std::string_view program = args.front();
  std::filesystem::path socket_directory;
  std::string_view instance = "default";
  std::size_t i = 1;
  for (; i < args.size() && std::string_view{args[i]}.starts_with("--"); ++i) {
    const std::string_view option = args[i];
    if (option == "--help") {
      std::print(usage, program);
      return EXIT_SUCCESS;
    }
    if (i + 1 >= args.size()) {
      throw std::invalid_argument{std::format("{} needs a value", option)};
    }
    if (option == "--instance") {
      instance = args[++i];
    } else if (option == "--socket-dir") {
      socket_directory = args[++i];
    } else {
      throw std::invalid_argument{std::format("unknown option '{}' (see --help)", option)};
    }
  }
  if (i >= args.size()) {
    throw std::invalid_argument{"no command given (see --help)"};
  }
  if (socket_directory.empty()) {
    socket_directory = sim::runtime::default_socket_directory(instance);
  }

  const std::vector<std::byte> request = make_request(args[i], args.subspan(i + 1));
  const std::filesystem::path socket = socket_directory / "sim.sock";
  UniqueFd fd;
  try {
    fd = sim::ipc::connect_to(socket);
  } catch (const std::system_error&) {
    throw std::runtime_error{std::format("could not connect to {}; is the simulator running?", socket.native())};
  }
  if (::send(fd.get(), request.data(), request.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(request.size())) {
    throw std::system_error{errno, std::generic_category(), "could not send the request"};
  }

  pollfd entry{.fd = fd.get(), .events = POLLIN, .revents = 0};
  sim_ctl_reply reply{};
  if (::poll(&entry, 1, reply_timeout_ms) != 1 ||
      ::recv(fd.get(), &reply, sizeof reply, 0) != static_cast<ssize_t>(sizeof reply) ||
      reply.header.type != SIM_CTL_REPLY) {
    throw std::runtime_error{"the simulator didn't answer"};
  }
  if (reply.status != SIM_OK) {
    std::println(stderr, "error: {}", reply.message);
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run({argv, static_cast<std::size_t>(argc)});
  } catch (const std::exception& error) {
    std::println(stderr, "error: {}", error.what());
    return EXIT_FAILURE;
  }
}
