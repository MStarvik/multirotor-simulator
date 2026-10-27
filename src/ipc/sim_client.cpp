#include "ipc/sim_client.hpp"

#include <cerrno>
#include <cstdio>
#include <print>
#include <stdexcept>
#include <system_error>

#include <sys/socket.h>

namespace sim::ipc {

SimClient::SimClient(const std::filesystem::path& socket)
    : fd_{connect_to(socket)},
      buffer_(max_message_size),
      roster_{std::make_shared<const runtime::Roster>()},
      obstacles_{std::make_shared<const runtime::Obstacles>()} {
  snapshot_.roster = roster_;
  snapshot_.obstacles = obstacles_;
  send(to_bytes(make_header(SIM_CTL_SUBSCRIBE)));
}

void SimClient::request_state() { send(to_bytes(make_header(SIM_CTL_GET_STATE))); }

void SimClient::set_paused(bool paused) {
  send(to_bytes(sim_ctl_pause{.header = make_header(SIM_CTL_PAUSE), .paused = paused, .reserved = 0}));
}

void SimClient::reset() { send(to_bytes(make_header(SIM_CTL_RESET))); }

void SimClient::set_pilot_input(std::string_view vehicle, const runtime::PilotInput& input) {
  sim_ctl_pilot_input request{
      .header = make_header(SIM_CTL_PILOT_INPUT),
      .vehicle = {},
      .sticks = {input.roll, input.pitch, input.yaw, input.throttle},
  };
  write_name(vehicle, request.vehicle);
  send(to_bytes(request));
}

void SimClient::send(std::span<const std::byte> message) {
  // A full socket drops the request; a closed one shows in receive().
  ::send(fd_.get(), message.data(), message.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
}

bool SimClient::receive() {
  while (true) {
    const ssize_t length = ::recv(fd_.get(), buffer_.data(), buffer_.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (length < 0 && errno == EINTR) {
      continue;
    }
    if (length < 0 && errno == EAGAIN) {
      return true;
    }
    if (length <= 0) {
      return false;
    }
    if (static_cast<std::size_t>(length) > buffer_.size()) {
      std::println(stderr, "warning: dropped a {} byte message from the simulator, larger than the maximum", length);
      continue;
    }
    try {
      handle_message(std::span{buffer_}.first(static_cast<std::size_t>(length)));
    } catch (const std::invalid_argument& error) {
      std::println(stderr, "warning: dropped {} from the simulator", error.what());
    }
  }
}

void SimClient::handle_message(std::span<const std::byte> message) {
  if (message.size() < sizeof(sim_header)) {
    throw std::invalid_argument{"a message too short for a header"};
  }
  const auto header = read_as<sim_header>(message);
  if (header.version != SIM_PROTOCOL_VERSION) {
    throw std::invalid_argument{"a message of another protocol version"};
  }
  switch (header.type) {
    case SIM_VIEW_VEHICLES:
      roster_ = decode_vehicles(message);
      break;
    case SIM_VIEW_OBSTACLES:
      // Obstacles don't move with the vehicles, so they needn't wait for a state.
      obstacles_ = decode_obstacles(message);
      snapshot_.obstacles = obstacles_;
      break;
    case SIM_VIEW_STATE: {
      // States for vehicles that are already gone again are skipped.
      [[maybe_unused]] const bool current = decode_state(message, roster_, snapshot_);
      break;
    }
    case SIM_VIZ_MARKER:
    case SIM_VIZ_PATH:
      drawings_.drawings[header.id] = decode_drawing(message);
      ++drawings_.version;
      break;
    case SIM_VIZ_REMOVE:
      drawings_.drawings.erase(header.id);
      ++drawings_.version;
      break;
    default:
      break;  // Replies to requests, which a viewer doesn't need.
  }
}

}  // namespace sim::ipc
