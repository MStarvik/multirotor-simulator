#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <span>
#include <stop_token>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "ipc/messages.hpp"
#include "ipc/unix_socket.hpp"
#include "runtime/sim_runner.hpp"
#include "runtime/triple_buffer.hpp"
#include "sim/multirotor.hpp"

namespace sim::ipc {

// Serves the simulator socket (see multirotor_sim/sim.h) on its own thread, so
// clients are never blocked by the simulation or a slow viewer. Keeps the drawings
// clients send, each client's own, until they remove them or disconnect. Turns
// requests into commands for the runner, and answers them once applied. Keeps
// subscribers up to date with the vehicles and drawings, and answers requests for
// the state of the vehicles.
class SimServer {
 public:
  // The runner must outlive the server, and not be running yet. Spawned vehicles
  // take what they don't specify from the default frame.
  SimServer(std::filesystem::path socket_path, runtime::SimRunner& runner, const SymmetricFrame& default_frame);
  ~SimServer();

  SimServer(const SimServer&) = delete;
  SimServer& operator=(const SimServer&) = delete;

  // Cheap; compare with the scene's version to see if anything changed.
  [[nodiscard]] std::uint64_t drawings_version() const { return version_.load(std::memory_order_acquire); }
  [[nodiscard]] VizScene drawings() const;

 private:
  struct Client {
    UniqueFd fd;
    std::uint64_t id = 0;  // Numbered from 1 in the order clients connected.
    bool subscribed = false;
    // Subscribers only: whether the current vehicles and obstacles were sent, and the
    // drawings that changed since they were last sent.
    bool roster_sent = false;
    std::uint64_t sent_roster_version = 0;
    bool obstacles_sent = false;
    std::uint64_t sent_obstacles_version = 0;
    std::set<std::uint32_t> changed_drawings{};
    bool blocked = false;  // Its socket was full; wait until it has room.
  };
  struct Reply;
  class ReplyQueue;

  void serve(const std::stop_token& stop);
  void receive(Client& client, std::span<std::byte> buffer);
  void handle_message(Client& client, std::span<const std::byte> message);
  void handle_drawing(const Client& client, std::span<const std::byte> message);
  void handle_request(Client& client, std::uint32_t type, std::span<const std::byte> message);
  // Applies the command, then answers the request if it asked for an answer.
  void submit(const Client& client, std::uint32_t request_id, runtime::Command command);
  void answer(const Client& client, std::uint32_t request_id, runtime::Status status, std::string_view message);
  // Throws std::invalid_argument for an invalid request.
  [[nodiscard]] runtime::VehicleSpec spawn_spec(const ::sim_ctl_spawn_vehicle& request) const;
  void send_replies();
  // Sends a subscriber what changed since it was last sent, while its socket has room.
  void update(Client& client, const runtime::Snapshot& snapshot);
  // Returns false, and marks the client blocked, if its socket is full.
  [[nodiscard]] bool send_to(Client& client, std::span<const std::byte> message);
  void disconnect(Client& client);
  // Call with mutex_ held.
  void set_drawing(const Client& client, std::uint32_t client_drawing_id, VizDrawing drawing);
  void remove_drawing(const Client& client, std::uint32_t client_drawing_id);
  void remove_drawings_of(const Client& client);
  void changed(std::uint32_t drawing);

  ListeningSocket listener_;
  runtime::SimRunner& runner_;
  runtime::TripleBuffer<runtime::Snapshot>& snapshots_;
  SymmetricFrame default_frame_;
  // Shared with the runner's completions, which may outlive the server.
  std::shared_ptr<ReplyQueue> replies_;

  // Only touched by the server's thread.
  std::vector<Client> clients_;
  std::uint64_t next_client_id_ = 1;
  // Drawing ids as clients chose them, by client, to the ids subscribers know them by.
  std::map<std::pair<std::uint64_t, std::uint32_t>, std::uint32_t> drawing_ids_;
  std::uint32_t next_drawing_id_ = 1;

  mutable std::mutex mutex_;
  VizScene scene_;  // Guarded by mutex_.
  std::atomic<std::uint64_t> version_{0};
  std::jthread thread_;  // Last, so it stops before the members it uses are destroyed.
};

}  // namespace sim::ipc
