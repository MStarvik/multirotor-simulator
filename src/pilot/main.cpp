#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <memory>
#include <numbers>
#include <print>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/socket.h>

#include <multirotor_sim/sitl.h>

#include "control/attitude_controller.hpp"
#include "ipc/unix_socket.hpp"
#include "runtime/options.hpp"

// A flight controller for the vehicles named by the options, over their SITL sockets
// (see multirotor_sim/sitl.h): the attitude controller in src/control, flown with the
// sticks the simulator passes on, such as a viewer's keyboard, or with --demo a
// scripted flight. Waits for the simulator if it isn't running yet, and exits once
// every vehicle's socket has closed, as when the simulator exits.
//
// With --demo, prints the state of each vehicle once per simulated second, and exits
// with failure if the state stops being finite.

namespace {

using namespace sim;

constexpr auto connect_retry_interval = std::chrono::milliseconds{100};

struct ScriptStep {
  double start;  // Simulation time, s.
  runtime::PilotInput input;
};

// Take off, hover, fly forward, hover.
constexpr std::array demo_script{
    ScriptStep{.start = 0.0, .input = {.throttle = 0.5}},
    ScriptStep{.start = 3.0, .input = {}},
    ScriptStep{.start = 5.0, .input = {.pitch = 0.3}},
    ScriptStep{.start = 7.0, .input = {}},
};

// From the time alone, so that it starts over when the simulation is reset.
runtime::PilotInput demo_input(double time) {
  runtime::PilotInput input;
  for (const ScriptStep& step : demo_script) {
    if (step.start <= time) {
      input = step.input;
    }
  }
  return input;
}

struct Vehicle {
  std::string name;
  std::filesystem::path socket;
  ipc::UniqueFd fd;  // Closed once the simulator has closed it.
  std::unique_ptr<control::AttitudeController> controller;
  std::uint64_t step_ns = 0;
};

Vec3 to_vec3(const double (&v)[3]) { return {v[0], v[1], v[2]}; }

double degrees(double radians) { return radians * 180.0 / std::numbers::pi; }

// The vehicle the hello describes, as far as the controller needs to know it.
MultirotorModel to_model(const sim_sitl_hello& hello) {
  if (hello.rotor_count > SIM_SITL_MAX_ROTORS) {
    throw std::runtime_error{std::format("the simulator describes a vehicle with {} rotors", hello.rotor_count)};
  }
  MultirotorModel model;
  model.body.mass = hello.mass;
  model.body.inertia = to_vec3(hello.inertia);
  model.propulsion.thrust_coefficient = hello.thrust_coefficient;
  model.propulsion.torque_coefficient = hello.torque_coefficient;
  model.propulsion.max_rotor_speed = hello.max_rotor_speed;
  for (std::uint32_t i = 0; i < hello.rotor_count; ++i) {
    model.rotors.push_back({.position = to_vec3(hello.rotors[i].position), .spin = hello.rotors[i].spin});
  }
  return model;
}

// Connects, waiting for the simulator if its socket isn't there yet, and reads the
// hello.
void connect(Vehicle& vehicle) {
  bool said_waiting = false;
  while (!vehicle.fd) {
    try {
      vehicle.fd = ipc::connect_to(vehicle.socket);
    } catch (const std::system_error&) {
      if (!std::exchange(said_waiting, true)) {
        std::println("Vehicle {}: waiting for the simulator on {}", vehicle.name, vehicle.socket.native());
      }
      std::this_thread::sleep_for(connect_retry_interval);
    }
  }

  sim_sitl_hello hello{};
  if (::recv(vehicle.fd.get(), &hello, sizeof hello, 0) != static_cast<ssize_t>(sizeof hello) ||
      hello.header.type != SIM_SITL_HELLO || hello.header.version != SIM_SITL_PROTOCOL_VERSION) {
    throw std::runtime_error{std::format("vehicle {}: expected a hello of protocol version {}", vehicle.name,
                                         SIM_SITL_PROTOCOL_VERSION)};
  }
  if (hello.step_ns == 0) {
    throw std::runtime_error{std::format("vehicle {}: the simulator's step is 0", vehicle.name)};
  }
  vehicle.controller = std::make_unique<control::AttitudeController>(to_model(hello));
  vehicle.step_ns = hello.step_ns;
  std::println("Vehicle {}: connected", vehicle.name);
}

// With several vehicles, each line starts with the vehicle's name, padded to
// `name_width`.
void print_state(double time, const VehicleState& state, double pressure, const std::string& name,
                 std::size_t name_width) {
  const Euler attitude = to_euler(state.attitude);
  std::println(
      "{:{}}{:6.1f} s  N {:7.2f}  E {:7.2f}  alt {:6.2f} m  roll {:6.1f}°  pitch {:6.1f}°  yaw {:6.1f}°  "
      "baro {:9.1f} Pa",
      name_width > 0 ? name + "  " : "", name_width > 0 ? name_width + 2 : 0, time, state.position.x,
      state.position.y, 0.0 - state.position.z, degrees(attitude.roll), degrees(attitude.pitch),
      degrees(attitude.yaw), pressure);
}

// Answers one sensors message. Returns false if the vehicle's state has diverged.
bool fly(Vehicle& vehicle, const sim_sitl_sensors& sensors, const runtime::Options& options,
         std::size_t name_width) {
  const VehicleState truth{
      .position = to_vec3(sensors.truth.position),
      .velocity = to_vec3(sensors.truth.velocity),
      .attitude = {sensors.truth.attitude[0], sensors.truth.attitude[1], sensors.truth.attitude[2],
                   sensors.truth.attitude[3]},
      .angular_velocity = to_vec3(sensors.truth.angular_velocity),
      .on_ground = sensors.truth.position[2] >= 0.0,
  };
  const SensorReadings readings{
      .imu = {.specific_force = to_vec3(sensors.accelerometer), .angular_velocity = to_vec3(sensors.gyroscope)},
      .barometer = {.pressure = sensors.pressure, .temperature = sensors.temperature},
      .barometer_fresh = (sensors.flags & SIM_SITL_BAROMETER_FRESH) != 0,
  };
  const double time = static_cast<double>(sensors.time_ns) * 1e-9;

  if (options.demo && sensors.time_ns % 1'000'000'000 == 0) {
    print_state(time, truth, sensors.pressure, vehicle.name, name_width);
  }
  if (!is_finite(truth.position) || !is_finite(truth.attitude)) {
    std::println(stderr, "error: the simulation of vehicle {} diverged", vehicle.name);
    return false;
  }

  if ((sensors.flags & SIM_SITL_RESET) != 0) {
    vehicle.controller->reset();
  }
  vehicle.controller->set_pilot_input(
      options.demo ? demo_input(time)
                   : runtime::PilotInput{.roll = sensors.pilot[0], .pitch = sensors.pilot[1],
                                         .yaw = sensors.pilot[2], .throttle = sensors.pilot[3]});
  const MotorCommands commands = vehicle.controller
                                     ->update({
                                         .step = sensors.time_ns / vehicle.step_ns,
                                         .time = time,
                                         .step_size = static_cast<double>(vehicle.step_ns) * 1e-9,
                                         .sensors = readings,
                                         .truth = truth,
                                     })
                                     .value_or(MotorCommands{});

  sim_sitl_actuators actuators{};
  actuators.header = {.type = SIM_SITL_ACTUATORS, .version = SIM_SITL_PROTOCOL_VERSION};
  actuators.sequence = sensors.sequence;
  std::copy_n(commands.begin(), std::min<std::size_t>(commands.size(), SIM_SITL_MAX_ROTORS), actuators.motors);
  // A vanished simulator shows up as the socket closing.
  ::send(vehicle.fd.get(), &actuators, sizeof actuators, MSG_NOSIGNAL);
  return true;
}

int run(const runtime::Options& options) {
  std::vector<Vehicle> vehicles;
  for (const std::string& name : options.vehicles) {
    vehicles.push_back({.name = name,
                        .socket = options.socket_directory / "vehicles" / std::format("{}.sock", name),
                        .fd = {},
                        .controller = nullptr,
                        .step_ns = 0});
  }
  for (Vehicle& vehicle : vehicles) {
    connect(vehicle);
  }
  std::size_t name_width = 0;
  if (vehicles.size() > 1) {
    name_width = std::ranges::max(options.vehicles, {}, &std::string::size).size();
  }

  std::vector<pollfd> entries;
  while (true) {
    if (std::ranges::none_of(vehicles, [](const Vehicle& v) { return static_cast<bool>(v.fd); })) {
      return EXIT_SUCCESS;
    }
    entries.clear();
    for (const Vehicle& vehicle : vehicles) {
      entries.push_back({.fd = vehicle.fd.get(), .events = POLLIN, .revents = 0});  // Ignored if closed (-1).
    }
    if (::poll(entries.data(), entries.size(), -1) < 0) {
      continue;  // Interrupted.
    }

    for (std::size_t i = 0; i < vehicles.size(); ++i) {
      Vehicle& vehicle = vehicles[i];
      if (entries[i].revents == 0) {
        continue;
      }
      sim_sitl_sensors sensors{};
      // MSG_TRUNC makes recv return the real length of an oversized message.
      const ssize_t length = ::recv(vehicle.fd.get(), &sensors, sizeof sensors, MSG_TRUNC);
      if (length < 0 && errno == EINTR) {
        continue;
      }
      if (length <= 0) {
        vehicle.fd.reset();  // The simulator exited, or removed the vehicle.
        continue;
      }
      if (length != static_cast<ssize_t>(sizeof sensors) || sensors.header.type != SIM_SITL_SENSORS ||
          sensors.header.version != SIM_SITL_PROTOCOL_VERSION) {
        throw std::runtime_error{std::format("vehicle {}: unexpected message from the simulator", vehicle.name)};
      }
      if (!fly(vehicle, sensors, options, name_width)) {
        return EXIT_FAILURE;
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    return options ? run(*options) : EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::println(stderr, "error: {}", error.what());
    return EXIT_FAILURE;
  }
}
