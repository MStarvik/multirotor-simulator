#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include "ipc/messages.hpp"
#include "ipc/unix_socket.hpp"
#include "runtime/flight_controller.hpp"
#include "runtime/sim_runner.hpp"

namespace sim::ipc {

// A viewer's connection to a simulator, over its simulator socket. Subscribes, and
// keeps a copy of what the simulator shows: its vehicles and their latest state,
// and everyone's drawings. Never blocks: requests that don't fit in the socket are
// dropped, which only happens if the simulator falls far behind.
class SimClient {
 public:
  // Throws std::system_error if no simulator listens there.
  explicit SimClient(const std::filesystem::path& socket);

  [[nodiscard]] int fd() const { return fd_.get(); }

  // Asks for the state of the vehicles, which a later receive() picks up.
  void request_state();
  // Reads everything that arrived. Returns false once the simulator has gone away.
  [[nodiscard]] bool receive();

  // The latest state, with the vehicles it refers to. Without vehicles until the
  // first state arrives.
  [[nodiscard]] const runtime::Snapshot& snapshot() const { return snapshot_; }
  [[nodiscard]] const VizScene& drawings() const { return drawings_; }

  void set_paused(bool paused);
  void reset();
  void set_pilot_input(std::string_view vehicle, const runtime::PilotInput& input);

 private:
  void send(std::span<const std::byte> message);
  void handle_message(std::span<const std::byte> message);

  UniqueFd fd_;
  std::vector<std::byte> buffer_;
  std::shared_ptr<const runtime::Roster> roster_;  // The latest; the snapshot's may be older.
  runtime::Snapshot snapshot_;
  VizScene drawings_;
};

}  // namespace sim::ipc
