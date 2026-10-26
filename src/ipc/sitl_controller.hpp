#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string_view>

#include <multirotor_sim/sitl.h>

#include "ipc/unix_socket.hpp"
#include "runtime/flight_controller.hpp"
#include "sim/multirotor.hpp"

namespace sim::ipc {

// An external flight controller connected over the SITL socket (see
// multirotor_sim/sitl.h). Each update() sends the step's sensor readings once and
// waits briefly for the answer; until it arrives, the simulation holds.
class SitlController final : public runtime::FlightController {
 public:
  SitlController(const MultirotorModel& model, double step_size, std::filesystem::path socket_path);

  void reset() override { reset_pending_ = true; }
  void set_pilot_input(const runtime::PilotInput& input) override { pilot_ = input; }
  [[nodiscard]] std::optional<MotorCommands> update(const runtime::ControllerInput& input) override;

 private:
  [[nodiscard]] bool accept_client();
  [[nodiscard]] bool send_sensors(const runtime::ControllerInput& input);
  [[nodiscard]] std::optional<MotorCommands> receive_actuators();
  void disconnect(std::string_view reason);

  sim_sitl_hello hello_{};
  std::size_t rotor_count_;
  std::uint64_t step_ns_;
  ListeningSocket listener_;
  UniqueFd client_;

  runtime::PilotInput pilot_;
  bool reset_pending_ = false;
  bool awaiting_answer_ = false;  // Sensors for the current step were sent.
  std::uint64_t sequence_ = 0;
};

}  // namespace sim::ipc
