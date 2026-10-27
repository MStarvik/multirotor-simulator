#include "ipc/sim_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <print>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sim::ipc {
namespace {

constexpr int poll_timeout_ms = 100;  // Bounds how long stopping, and noticing new vehicles, takes.
// Room for several of the largest messages, so a subscriber always gets the vehicles.
constexpr int send_buffer_size = static_cast<int>(4 * max_message_size);

sim_status to_wire(runtime::Status status) {
  switch (status) {
    case runtime::Status::ok: return SIM_OK;
    case runtime::Status::invalid: return SIM_INVALID;
    case runtime::Status::not_found: return SIM_NOT_FOUND;
    case runtime::Status::exists: return SIM_EXISTS;
    case runtime::Status::unsupported: return SIM_UNSUPPORTED;
    case runtime::Status::failed: return SIM_FAILED;
  }
  return SIM_FAILED;
}

sim_ctl_reply make_reply(std::uint32_t request_id, runtime::Status status, std::string_view message) {
  sim_ctl_reply reply{};
  reply.header = make_header(SIM_CTL_REPLY, request_id);
  reply.status = to_wire(status);
  const std::size_t length = std::min(message.size(), sizeof reply.message - 1);
  std::memcpy(reply.message, message.data(), length);
  return reply;
}

}  // namespace

struct SimServer::Reply {
  std::uint64_t client;
  sim_ctl_reply message;
};

// Replies from the simulation thread, waiting to be sent by the server's thread,
// which an eventfd wakes up.
class SimServer::ReplyQueue {
 public:
  ReplyQueue() : event_{::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)} {
    if (!event_) {
      throw std::system_error{errno, std::generic_category(), "eventfd"};
    }
  }

  [[nodiscard]] int fd() const { return event_.get(); }

  void post(Reply reply) {
    {
      const std::lock_guard lock{mutex_};
      replies_.push_back(reply);
    }
    const std::uint64_t one = 1;
    [[maybe_unused]] const ssize_t written = ::write(event_.get(), &one, sizeof one);
  }

  std::vector<Reply> take() {
    std::uint64_t count = 0;
    [[maybe_unused]] const ssize_t read = ::read(event_.get(), &count, sizeof count);
    const std::lock_guard lock{mutex_};
    return std::exchange(replies_, {});
  }

 private:
  UniqueFd event_;
  std::mutex mutex_;
  std::vector<Reply> replies_;  // Guarded by mutex_.
};

SimServer::SimServer(std::filesystem::path socket_path, runtime::SimRunner& runner,
                     const SymmetricFrame& default_frame)
    : listener_{std::move(socket_path)},
      runner_{runner},
      snapshots_{runner.add_snapshot_reader()},
      default_frame_{default_frame},
      replies_{std::make_shared<ReplyQueue>()},
      thread_{[this](std::stop_token stop) { serve(stop); }} {
  std::println("Accepting drawings and requests on {}", listener_.path().native());
}

SimServer::~SimServer() = default;

VizScene SimServer::drawings() const {
  const std::lock_guard lock{mutex_};
  return scene_;
}

void SimServer::serve(const std::stop_token& stop) {
  std::vector<pollfd> poll_entries;
  std::vector<std::byte> buffer(max_message_size);
  constexpr std::size_t first_client_entry = 2;  // After the listener and the reply queue.

  while (!stop.stop_requested()) {
    poll_entries.clear();
    poll_entries.push_back({.fd = listener_.fd(), .events = POLLIN, .revents = 0});
    poll_entries.push_back({.fd = replies_->fd(), .events = POLLIN, .revents = 0});
    for (const Client& client : clients_) {
      const short events = client.blocked ? POLLIN | POLLOUT : POLLIN;
      poll_entries.push_back({.fd = client.fd.get(), .events = events, .revents = 0});
    }
    ::poll(poll_entries.data(), poll_entries.size(), poll_timeout_ms);

    for (std::size_t i = 0; i < clients_.size(); ++i) {
      const short revents = poll_entries[first_client_entry + i].revents;
      if ((revents & POLLOUT) != 0) {
        clients_[i].blocked = false;
      }
      if ((revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
        receive(clients_[i], buffer);
      }
    }
    if ((poll_entries[1].revents & POLLIN) != 0) {
      send_replies();
    }

    // Also every poll timeout, to notice vehicles and obstacles changing.
    const runtime::Snapshot& snapshot = snapshots_.latest();
    for (Client& client : clients_) {
      update(client, snapshot);
    }
    std::erase_if(clients_, [](const Client& client) { return !client.fd; });

    if ((poll_entries[0].revents & POLLIN) != 0) {
      if (UniqueFd fd = listener_.accept()) {
        ::setsockopt(fd.get(), SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof send_buffer_size);
        clients_.push_back({.fd = std::move(fd), .id = next_client_id_++});
      }
    }
  }
}

void SimServer::receive(Client& client, std::span<std::byte> buffer) {
  // Drain everything this client has sent.
  while (client.fd) {
    const ssize_t length = ::recv(client.fd.get(), buffer.data(), buffer.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (length < 0 && (errno == EAGAIN || errno == EINTR)) {
      return;
    }
    if (length <= 0) {
      disconnect(client);
      return;
    }
    if (static_cast<std::size_t>(length) > buffer.size()) {
      std::println(stderr, "sim.sock: dropped a {} byte message, larger than the maximum", length);
      continue;
    }
    handle_message(client, buffer.first(static_cast<std::size_t>(length)));
  }
}

void SimServer::handle_message(Client& client, std::span<const std::byte> message) {
  if (message.size() < sizeof(sim_header)) {
    std::println(stderr, "sim.sock: dropped a message too short for a header");
    return;
  }
  const auto header = read_as<sim_header>(message);
  if (header.version != SIM_PROTOCOL_VERSION) {
    std::println(stderr, "sim.sock: dropped a message with protocol version {}", header.version);
    return;
  }
  if (header.type >= SIM_CTL_PAUSE) {
    handle_request(client, header.type, message);
  } else {
    handle_drawing(client, message);
  }
}

void SimServer::handle_drawing(const Client& client, std::span<const std::byte> message) {
  const auto header = read_as<sim_header>(message);
  const std::lock_guard lock{mutex_};
  switch (header.type) {
    case SIM_VIZ_MARKER:
    case SIM_VIZ_PATH:
      try {
        set_drawing(client, header.id, decode_drawing(message));
      } catch (const std::invalid_argument& error) {
        std::println(stderr, "sim.sock: dropped {}", error.what());
      }
      break;
    case SIM_VIZ_REMOVE:
      remove_drawing(client, header.id);
      break;
    case SIM_VIZ_CLEAR:
      remove_drawings_of(client);
      break;
    default:
      std::println(stderr, "sim.sock: dropped a message of unknown type {}", header.type);
      break;
  }
}

void SimServer::handle_request(Client& client, std::uint32_t type, std::span<const std::byte> message) {
  const auto header = read_as<sim_header>(message);
  const auto reject = [&](std::string_view why) { answer(client, header.id, runtime::Status::invalid, why); };
  const auto expect_size = [&](std::size_t size) {
    if (message.size() != size) {
      reject(std::format("a message of type {} must be {} bytes, not {}", type, size, message.size()));
      return false;
    }
    return true;
  };

  switch (type) {
    case SIM_CTL_PAUSE:
      if (expect_size(sizeof(sim_ctl_pause))) {
        submit(client, header.id, runtime::SetPaused{.paused = read_as<sim_ctl_pause>(message).paused != 0});
      }
      break;
    case SIM_CTL_RESET:
      if (expect_size(sizeof(sim_header))) {
        submit(client, header.id, runtime::Reset{});
      }
      break;
    case SIM_CTL_SET_SPEED:
      if (expect_size(sizeof(sim_ctl_set_speed))) {
        submit(client, header.id, runtime::SetSpeed{.speed = read_as<sim_ctl_set_speed>(message).speed});
      }
      break;
    case SIM_CTL_PILOT_INPUT:
      if (expect_size(sizeof(sim_ctl_pilot_input))) {
        const auto request = read_as<sim_ctl_pilot_input>(message);
        const std::string vehicle = read_name(request.vehicle);
        if (vehicle.empty()) {
          reject("no vehicle named");
          break;
        }
        const auto clamp = [](double stick) { return std::clamp(stick, -1.0, 1.0); };
        submit(client, header.id,
               runtime::SetPilotInput{.vehicle = vehicle,
                                      .input = {.roll = clamp(request.sticks[0]),
                                                .pitch = clamp(request.sticks[1]),
                                                .yaw = clamp(request.sticks[2]),
                                                .throttle = clamp(request.sticks[3])}});
      }
      break;
    case SIM_CTL_SPAWN_VEHICLE:
      if (expect_size(sizeof(sim_ctl_spawn_vehicle))) {
        try {
          submit(client, header.id, runtime::SpawnVehicle{spawn_spec(read_as<sim_ctl_spawn_vehicle>(message))});
        } catch (const std::invalid_argument& error) {
          reject(error.what());
        }
      }
      break;
    case SIM_CTL_REMOVE_VEHICLE:
      if (expect_size(sizeof(sim_ctl_remove_vehicle))) {
        submit(client, header.id,
               runtime::RemoveVehicle{.name = read_name(read_as<sim_ctl_remove_vehicle>(message).name)});
      }
      break;
    case SIM_CTL_SET_OBSTACLE:
      if (expect_size(sizeof(sim_ctl_set_obstacle))) {
        try {
          submit(client, header.id,
                 runtime::SetObstacle{decode_obstacle(read_as<sim_ctl_set_obstacle>(message).obstacle)});
        } catch (const std::invalid_argument& error) {
          reject(error.what());
        }
      }
      break;
    case SIM_CTL_REMOVE_OBSTACLE:
      if (expect_size(sizeof(sim_ctl_remove_obstacle))) {
        submit(client, header.id,
               runtime::RemoveObstacle{.name = read_name(read_as<sim_ctl_remove_obstacle>(message).name)});
      }
      break;
    case SIM_CTL_SUBSCRIBE:
      if (expect_size(sizeof(sim_header))) {
        // Everything is news to a new subscriber.
        client.subscribed = true;
        client.roster_sent = false;
        client.obstacles_sent = false;
        {
          const std::lock_guard lock{mutex_};
          for (const auto& [id, drawing] : scene_.drawings) {
            client.changed_drawings.insert(id);
          }
        }
        answer(client, header.id, runtime::Status::ok, {});
      }
      break;
    case SIM_CTL_GET_STATE:
      if (expect_size(sizeof(sim_header))) {
        // The vehicles first, so that the state can be matched to them.
        const runtime::Snapshot& snapshot = snapshots_.latest();
        update(client, snapshot);
        if (!client.blocked && client.fd) {
          // Dropped if there's no room: the client asks again for a newer one anyway.
          [[maybe_unused]] const bool sent = send_to(client, encode_state(header.id, snapshot));
        }
      }
      break;
    default:
      reject(std::format("unknown message type {}", type));
      break;
  }
}

runtime::VehicleSpec SimServer::spawn_spec(const ::sim_ctl_spawn_vehicle& request) const {
  SymmetricFrame frame = default_frame_;
  if (request.rotor_count != 0) {
    frame.rotor_count = static_cast<int>(std::min(request.rotor_count, 1000u));
  }
  if (request.arm_length != 0.0) {
    frame.arm_length = request.arm_length;
  }
  if (request.mass != 0.0) {
    frame.body.mass = request.mass;
  }
  const Vec3 position{request.position[0], request.position[1], request.position[2]};
  if (!is_finite(position) || !std::isfinite(request.yaw) || position.z > 0.0) {
    throw std::invalid_argument{"the position and yaw must be finite, and the position not below the ground"};
  }
  if (request.controller != SIM_CONTROLLER_BUILT_IN && request.controller != SIM_CONTROLLER_SITL) {
    throw std::invalid_argument{std::format("unknown controller {}", request.controller)};
  }
  return {
      .name = read_name(request.name),
      .model = make_multirotor(frame),  // Throws std::invalid_argument for an impossible frame.
      .initial_state = {.position = position,
                        .attitude = from_rotation_vector({0.0, 0.0, request.yaw}),
                        .on_ground = position.z == 0.0},
      .controller = request.controller == SIM_CONTROLLER_SITL ? runtime::ControllerKind::sitl
                                                              : runtime::ControllerKind::built_in,
  };
}

void SimServer::submit(const Client& client, std::uint32_t request_id, runtime::Command command) {
  if (request_id == 0) {
    runner_.push(std::move(command));
    return;
  }
  runner_.push(std::move(command),
               [replies = replies_, client = client.id, request_id](const runtime::CommandResult& result) {
                 replies->post({.client = client, .message = make_reply(request_id, result.status, result.message)});
               });
}

void SimServer::answer(const Client& client, std::uint32_t request_id, runtime::Status status,
                       std::string_view message) {
  if (request_id != 0) {
    replies_->post({.client = client.id, .message = make_reply(request_id, status, message)});
  }
}

void SimServer::send_replies() {
  for (const Reply& reply : replies_->take()) {
    const auto client = std::ranges::find(clients_, reply.client, &Client::id);
    if (client == clients_.end() || !client->fd) {
      continue;  // It disconnected meanwhile.
    }
    // A client that doesn't read its replies loses them rather than blocking us.
    ::send(client->fd.get(), &reply.message, sizeof reply.message, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
}

void SimServer::update(Client& client, const runtime::Snapshot& snapshot) {
  if (!client.subscribed || client.blocked || !client.fd) {
    return;
  }
  const runtime::Roster& roster = *snapshot.roster;
  if (!client.roster_sent || client.sent_roster_version != roster.version) {
    if (!send_to(client, encode_vehicles(roster))) {
      return;
    }
    client.roster_sent = true;
    client.sent_roster_version = roster.version;
  }
  const runtime::Obstacles& obstacles = *snapshot.obstacles;
  if (!client.obstacles_sent || client.sent_obstacles_version != obstacles.version) {
    if (!send_to(client, encode_obstacles(obstacles))) {
      return;
    }
    client.obstacles_sent = true;
    client.sent_obstacles_version = obstacles.version;
  }

  // Whatever each changed drawing is now, which may be gone.
  while (!client.changed_drawings.empty()) {
    const std::uint32_t id = *client.changed_drawings.begin();
    std::vector<std::byte> message;
    {
      const std::lock_guard lock{mutex_};
      const auto drawing = scene_.drawings.find(id);
      message = drawing != scene_.drawings.end() ? encode_drawing(id, drawing->second)
                                                 : to_bytes(make_header(SIM_VIZ_REMOVE, id));
    }
    if (!send_to(client, message)) {
      return;
    }
    client.changed_drawings.erase(client.changed_drawings.begin());
  }
}

bool SimServer::send_to(Client& client, std::span<const std::byte> message) {
  if (::send(client.fd.get(), message.data(), message.size(), MSG_DONTWAIT | MSG_NOSIGNAL) >= 0) {
    return true;
  }
  if (errno == EAGAIN) {
    client.blocked = true;
  } else {
    disconnect(client);
  }
  return false;
}

void SimServer::disconnect(Client& client) {
  client.fd.reset();
  const std::lock_guard lock{mutex_};
  remove_drawings_of(client);
}

void SimServer::set_drawing(const Client& client, std::uint32_t client_drawing_id, VizDrawing drawing) {
  const auto [entry, added] = drawing_ids_.try_emplace({client.id, client_drawing_id}, next_drawing_id_);
  if (added) {
    ++next_drawing_id_;
  }
  scene_.drawings[entry->second] = std::move(drawing);
  changed(entry->second);
}

void SimServer::remove_drawing(const Client& client, std::uint32_t client_drawing_id) {
  const auto entry = drawing_ids_.find({client.id, client_drawing_id});
  if (entry == drawing_ids_.end()) {
    return;
  }
  const std::uint32_t id = entry->second;
  drawing_ids_.erase(entry);
  scene_.drawings.erase(id);
  changed(id);
}

void SimServer::remove_drawings_of(const Client& client) {
  const auto first = drawing_ids_.lower_bound({client.id, 0});
  const auto last = drawing_ids_.lower_bound({client.id + 1, 0});
  for (auto entry = first; entry != last; ++entry) {
    scene_.drawings.erase(entry->second);
    changed(entry->second);
  }
  drawing_ids_.erase(first, last);
}

void SimServer::changed(std::uint32_t drawing) {
  for (Client& subscriber : clients_) {
    if (subscriber.subscribed) {
      subscriber.changed_drawings.insert(drawing);
    }
  }
  version_.store(++scene_.version, std::memory_order_release);
}

}  // namespace sim::ipc
