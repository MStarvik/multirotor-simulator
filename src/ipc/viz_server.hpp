#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <span>
#include <stop_token>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "ipc/unix_socket.hpp"

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

// Receives drawing commands (see multirotor_sim/viz.h) from any number of clients
// on its own thread, so clients are never blocked by a slow or hidden viewer. Each
// client has its own drawing ids, and its drawings go away when it disconnects.
class VizServer {
 public:
  explicit VizServer(std::filesystem::path socket_path);

  VizServer(const VizServer&) = delete;
  VizServer& operator=(const VizServer&) = delete;

  // Cheap; compare with the scene's version to see if anything changed.
  [[nodiscard]] std::uint64_t version() const { return version_.load(std::memory_order_acquire); }
  [[nodiscard]] VizScene scene() const;

 private:
  void serve(const std::stop_token& stop);
  void handle_message(std::uint64_t client, std::span<const std::byte> message);
  void remove_drawings_of(std::uint64_t client);
  void changed();  // Call with mutex_ held.

  ListeningSocket listener_;
  mutable std::mutex mutex_;
  VizScene scene_;  // Guarded by mutex_.
  std::atomic<std::uint64_t> version_{0};
  std::jthread thread_;  // Last, so it stops before the members it uses are destroyed.
};

}  // namespace sim::ipc
