#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <stop_token>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "ipc/unix_socket.hpp"
#include "runtime/sim_runner.hpp"

namespace sim::ipc {

using Float3 = std::array<float, 3>;

struct VizMarker {
  Float3 position;
  Float3 color;
  float size;
};

struct VizPath {
  std::vector<Float3> points;
  Float3 color;
  float width;
};

using VizDrawing = std::variant<VizMarker, VizPath>;

// Identifies a drawing: the connection that sent it, numbered from 1 in the order
// clients connected, and the id that client chose for it.
using VizDrawingKey = std::pair<std::uint64_t, std::uint32_t>;

struct VizScene {
  std::uint64_t version = 0;  // Increases with every change.
  std::map<VizDrawingKey, VizDrawing> drawings;
};

// Serves the simulator socket (see multirotor_sim/sim.h) on its own thread, so
// clients are never blocked by the simulation or a slow viewer. Keeps the drawings
// clients send, each client's own, until they remove them or disconnect. Turns
// requests into commands for the runner, and answers them once applied.
class SimServer {
 public:
  // The runner must outlive the server.
  SimServer(std::filesystem::path socket_path, runtime::SimRunner& runner);
  ~SimServer();

  SimServer(const SimServer&) = delete;
  SimServer& operator=(const SimServer&) = delete;

  // Cheap; compare with the scene's version to see if anything changed.
  [[nodiscard]] std::uint64_t drawings_version() const { return version_.load(std::memory_order_acquire); }
  [[nodiscard]] VizScene drawings() const;

 private:
  struct Client;
  struct Reply;
  class ReplyQueue;

  void serve(const std::stop_token& stop);
  void handle_message(Client& client, std::span<const std::byte> message);
  void handle_drawing(std::uint64_t client, std::uint32_t type, std::span<const std::byte> message);
  void handle_request(Client& client, std::uint32_t type, std::span<const std::byte> message);
  // Applies the command, then answers the request if it asked for an answer.
  void submit(const Client& client, std::uint32_t request_id, runtime::Command command);
  void send_replies(std::span<Client> clients);
  void remove_drawings_of(std::uint64_t client);
  void changed();  // Call with mutex_ held.

  ListeningSocket listener_;
  runtime::SimRunner& runner_;
  // Shared with the runner's completions, which may outlive the server.
  std::shared_ptr<ReplyQueue> replies_;
  mutable std::mutex mutex_;
  VizScene scene_;  // Guarded by mutex_.
  std::atomic<std::uint64_t> version_{0};
  std::jthread thread_;  // Last, so it stops before the members it uses are destroyed.
};

}  // namespace sim::ipc
