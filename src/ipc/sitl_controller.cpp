#include "ipc/sitl_controller.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <print>
#include <utility>

#include <poll.h>
#include <sys/socket.h>

namespace sim::ipc {
namespace {

// How long one update() waits for a connection or an answer before returning, so
// the simulation thread stays responsive to pausing and stopping.
constexpr int wait_timeout_ms = 50;

void copy(const Vec3& from, double (&to)[3]) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

template <typename Message>
bool send_message(int fd, const Message& message) {
  // MSG_NOSIGNAL: a vanished peer is an error to handle, not a SIGPIPE.
  return ::send(fd, &message, sizeof message, MSG_NOSIGNAL) == static_cast<ssize_t>(sizeof message);
}

}  // namespace

SitlController::SitlController(const MultirotorModel& model, double step_size,
                               std::filesystem::path socket_path)
    : rotor_count_{model.rotors.size()},
      step_ns_{static_cast<std::uint64_t>(std::llround(step_size * 1e9))},
      listener_{std::move(socket_path)} {
  hello_.header = {.type = SIM_SITL_HELLO, .version = SIM_SITL_PROTOCOL_VERSION};
  hello_.rotor_count = static_cast<std::uint32_t>(rotor_count_);
  hello_.step_ns = step_ns_;
  hello_.mass = model.body.mass;
  copy(model.body.inertia, hello_.inertia);
  hello_.thrust_coefficient = model.propulsion.thrust_coefficient;
  hello_.torque_coefficient = model.propulsion.torque_coefficient;
  hello_.max_rotor_speed = model.propulsion.max_rotor_speed;
  for (std::size_t i = 0; i < rotor_count_; ++i) {
    copy(model.rotors[i].position, hello_.rotors[i].position);
    hello_.rotors[i].spin = model.rotors[i].spin;
  }

  std::println("Waiting for a flight controller on {}", listener_.path().native());
}

std::optional<MotorCommands> SitlController::update(const runtime::ControllerInput& input) {
  if (!client_ && !accept_client()) {
    return std::nullopt;
  }
  if (!awaiting_answer_) {
    if (!send_sensors(input)) {
      return std::nullopt;
    }
    awaiting_answer_ = true;
  }

  auto commands = receive_actuators();
  if (commands) {
    awaiting_answer_ = false;
    ++sequence_;
  }
  return commands;
}

bool SitlController::accept_client() {
  if (wait_readable(listener_.fd(), wait_timeout_ms) == 0) {
    return false;
  }
  client_ = listener_.accept();
  if (!client_) {
    return false;
  }
  if (!send_message(client_.get(), hello_)) {
    disconnect("could not send hello");
    return false;
  }
  awaiting_answer_ = false;  // A new flight controller gets the current step again.
  std::println("Flight controller connected");
  return true;
}

bool SitlController::send_sensors(const runtime::ControllerInput& input) {
  const SensorReadings& sensors = input.sensors;
  const VehicleState& truth = input.truth;

  sim_sitl_sensors message{};
  message.header = {.type = SIM_SITL_SENSORS, .version = SIM_SITL_PROTOCOL_VERSION};
  message.flags = (sensors.barometer_fresh ? SIM_SITL_BAROMETER_FRESH : 0u) |
                  (reset_pending_ ? SIM_SITL_RESET : 0u);
  message.sequence = sequence_;
  message.time_ns = input.step * step_ns_;
  copy(sensors.imu.specific_force, message.accelerometer);
  copy(sensors.imu.angular_velocity, message.gyroscope);
  message.pressure = sensors.barometer.pressure;
  message.temperature = sensors.barometer.temperature;
  message.pilot[0] = pilot_.roll;
  message.pilot[1] = pilot_.pitch;
  message.pilot[2] = pilot_.yaw;
  message.pilot[3] = pilot_.throttle;
  copy(truth.position, message.truth.position);
  copy(truth.velocity, message.truth.velocity);
  message.truth.attitude[0] = truth.attitude.w;
  message.truth.attitude[1] = truth.attitude.x;
  message.truth.attitude[2] = truth.attitude.y;
  message.truth.attitude[3] = truth.attitude.z;
  copy(truth.angular_velocity, message.truth.angular_velocity);

  if (!send_message(client_.get(), message)) {
    disconnect("could not send sensor data");
    return false;
  }
  reset_pending_ = false;
  return true;
}

std::optional<MotorCommands> SitlController::receive_actuators() {
  if (wait_readable(client_.get(), wait_timeout_ms) == 0) {
    return std::nullopt;
  }

  sim_sitl_actuators message{};
  // MSG_TRUNC makes recv return the real length of an oversized message.
  const ssize_t length = ::recv(client_.get(), &message, sizeof message, MSG_TRUNC);
  if (length == 0) {
    disconnect("disconnected");
    return std::nullopt;
  }
  if (length < 0) {
    if (errno != EAGAIN && errno != EINTR) {
      disconnect("receive failed");
    }
    return std::nullopt;
  }
  if (length != static_cast<ssize_t>(sizeof message) || message.header.type != SIM_SITL_ACTUATORS) {
    disconnect("sent an unexpected message");
    return std::nullopt;
  }
  if (message.header.version != SIM_SITL_PROTOCOL_VERSION) {
    disconnect("uses a different protocol version");
    return std::nullopt;
  }
  if (message.sequence != sequence_) {
    disconnect("answered the wrong sequence number");
    return std::nullopt;
  }

  MotorCommands commands{};
  for (std::size_t i = 0; i < rotor_count_; ++i) {
    commands[i] = message.motors[i];
  }
  return commands;
}

void SitlController::disconnect(std::string_view reason) {
  std::println(stderr, "Flight controller {}; waiting for it to reconnect", reason);
  client_.reset();
  awaiting_answer_ = false;
}

}  // namespace sim::ipc
