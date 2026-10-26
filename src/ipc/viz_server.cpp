#include "ipc/viz_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <print>
#include <utility>

#include <poll.h>
#include <sys/socket.h>

#include <multirotor_sim/viz.h>

namespace sim::ipc {
namespace {

constexpr std::size_t max_message_size =
    sizeof(sim_viz_path) + SIM_VIZ_MAX_PATH_POINTS * sizeof(Float3);
constexpr int poll_timeout_ms = 100;  // Bounds how long stopping takes.

template <typename T>
T read_as(std::span<const std::byte> bytes) {
  T value;
  std::memcpy(&value, bytes.data(), sizeof value);  // memcpy, since the bytes may be unaligned.
  return value;
}

Float3 to_float3(const float (&v)[3]) { return {v[0], v[1], v[2]}; }

}  // namespace

VizServer::VizServer(std::filesystem::path socket_path)
    : listener_{std::move(socket_path)}, thread_{[this](std::stop_token stop) { serve(stop); }} {
  std::println("Accepting drawings on {}", listener_.path().native());
}

VizScene VizServer::scene() const {
  const std::lock_guard lock{mutex_};
  return scene_;
}

void VizServer::serve(const std::stop_token& stop) {
  std::vector<UniqueFd> clients;
  std::vector<pollfd> poll_entries;
  std::vector<std::byte> buffer(max_message_size);

  while (!stop.stop_requested()) {
    poll_entries.clear();
    poll_entries.push_back({.fd = listener_.fd(), .events = POLLIN, .revents = 0});
    for (const UniqueFd& client : clients) {
      poll_entries.push_back({.fd = client.get(), .events = POLLIN, .revents = 0});
    }
    if (::poll(poll_entries.data(), poll_entries.size(), poll_timeout_ms) <= 0) {
      continue;
    }

    for (std::size_t i = 0; i < clients.size(); ++i) {
      if (poll_entries[i + 1].revents == 0) {
        continue;
      }
      // Drain everything this client has sent.
      while (true) {
        const ssize_t length =
            ::recv(clients[i].get(), buffer.data(), buffer.size(), MSG_DONTWAIT | MSG_TRUNC);
        if (length < 0 && (errno == EAGAIN || errno == EINTR)) {
          break;
        }
        if (length <= 0) {
          clients[i].reset();  // Disconnected.
          break;
        }
        if (static_cast<std::size_t>(length) > buffer.size()) {
          std::println(stderr, "viz: dropped a {} byte message, larger than the maximum", length);
          continue;
        }
        handle_message(std::span{buffer}.first(static_cast<std::size_t>(length)));
      }
    }
    std::erase_if(clients, [](const UniqueFd& client) { return !client; });

    if ((poll_entries.front().revents & POLLIN) != 0) {
      if (UniqueFd client = listener_.accept()) {
        clients.push_back(std::move(client));
      }
    }
  }
}

void VizServer::handle_message(std::span<const std::byte> message) {
  if (message.size() < sizeof(sim_viz_header)) {
    std::println(stderr, "viz: dropped a message too short for a header");
    return;
  }
  const auto header = read_as<sim_viz_header>(message);
  if (header.version != SIM_VIZ_PROTOCOL_VERSION) {
    std::println(stderr, "viz: dropped a message with protocol version {}", header.version);
    return;
  }

  const std::lock_guard lock{mutex_};
  switch (header.type) {
    case SIM_VIZ_MARKER: {
      if (message.size() != sizeof(sim_viz_marker)) {
        std::println(stderr, "viz: dropped a marker of the wrong size");
        return;
      }
      const auto marker = read_as<sim_viz_marker>(message);
      scene_.drawings[header.id] = VizMarker{
          .position = to_float3(marker.position),
          .color = to_float3(marker.color),
          .size = marker.size,
      };
      break;
    }
    case SIM_VIZ_PATH: {
      if (message.size() < sizeof(sim_viz_path)) {
        std::println(stderr, "viz: dropped a path of the wrong size");
        return;
      }
      const auto path = read_as<sim_viz_path>(message);
      const auto points = message.subspan(sizeof(sim_viz_path));
      if (path.point_count > SIM_VIZ_MAX_PATH_POINTS || points.size() != path.point_count * sizeof(Float3)) {
        std::println(stderr, "viz: dropped a path whose size doesn't match its point count");
        return;
      }
      VizPath drawing{.points = std::vector<Float3>(path.point_count),
                      .color = to_float3(path.color),
                      .width = path.width};
      std::memcpy(drawing.points.data(), points.data(), points.size());
      scene_.drawings[header.id] = std::move(drawing);
      break;
    }
    case SIM_VIZ_REMOVE:
      scene_.drawings.erase(header.id);
      break;
    case SIM_VIZ_CLEAR:
      scene_.drawings.clear();
      break;
    default:
      std::println(stderr, "viz: dropped a message of unknown type {}", header.type);
      return;
  }
  version_.store(++scene_.version, std::memory_order_release);
}

}  // namespace sim::ipc
