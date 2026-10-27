#include "ipc/sim_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <print>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <multirotor_sim/sim.h>

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

// Up to the first NUL, or all of it.
std::string read_name(const char (&name)[SIM_NAME_SIZE]) {
  return std::string{name, ::strnlen(name, SIM_NAME_SIZE)};
}

// Removes all drawings of one client. Returns whether there were any.
bool erase_client(std::map<VizDrawingKey, VizDrawing>& drawings, std::uint64_t client) {
  const auto first = drawings.lower_bound({client, 0});
  const auto last = drawings.lower_bound({client + 1, 0});
  const bool any = first != last;
  drawings.erase(first, last);
  return any;
}

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

sim_ctl_reply make_reply(std::uint32_t request_id, sim_status status, std::string_view message) {
  sim_ctl_reply reply{};
  reply.header = {.type = SIM_CTL_REPLY, .version = SIM_PROTOCOL_VERSION, .id = request_id, .reserved = 0};
  reply.status = status;
  const std::size_t length = std::min(message.size(), sizeof reply.message - 1);
  std::memcpy(reply.message, message.data(), length);
  return reply;
}

}  // namespace

struct SimServer::Client {
  UniqueFd fd;
  std::uint64_t id;
};

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
  std::vector<Client> clients;
  std::uint64_t next_client_id = 1;
  std::vector<pollfd> poll_entries;
  std::vector<std::byte> buffer(max_message_size);
  constexpr std::size_t first_client_entry = 2;  // After the listener and the reply queue.

  while (!stop.stop_requested()) {
    poll_entries.clear();
    poll_entries.push_back({.fd = listener_.fd(), .events = POLLIN, .revents = 0});
    poll_entries.push_back({.fd = replies_->fd(), .events = POLLIN, .revents = 0});
    for (const Client& client : clients) {
      poll_entries.push_back({.fd = client.fd.get(), .events = POLLIN, .revents = 0});
    }
    if (::poll(poll_entries.data(), poll_entries.size(), poll_timeout_ms) <= 0) {
      continue;
    }

    for (std::size_t i = 0; i < clients.size(); ++i) {
      if (poll_entries[first_client_entry + i].revents == 0) {
        continue;
      }
      // Drain everything this client has sent.
      Client& client = clients[i];
      while (true) {
        const ssize_t length =
            ::recv(client.fd.get(), buffer.data(), buffer.size(), MSG_DONTWAIT | MSG_TRUNC);
        if (length < 0 && (errno == EAGAIN || errno == EINTR)) {
          break;
        }
        if (length <= 0) {
          client.fd.reset();  // Disconnected.
          remove_drawings_of(client.id);
          break;
        }
        if (static_cast<std::size_t>(length) > buffer.size()) {
          std::println(stderr, "sim.sock: dropped a {} byte message, larger than the maximum", length);
          continue;
        }
        handle_message(client, std::span{buffer}.first(static_cast<std::size_t>(length)));
      }
    }
    std::erase_if(clients, [](const Client& client) { return !client.fd; });

    if ((poll_entries[1].revents & POLLIN) != 0) {
      send_replies(clients);
    }
    if ((poll_entries[0].revents & POLLIN) != 0) {
      if (UniqueFd fd = listener_.accept()) {
        clients.push_back({.fd = std::move(fd), .id = next_client_id++});
      }
    }
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
    handle_drawing(client.id, header.type, message);
  }
}

void SimServer::handle_drawing(std::uint64_t client, std::uint32_t type, std::span<const std::byte> message) {
  const auto header = read_as<sim_header>(message);
  const std::lock_guard lock{mutex_};
  switch (type) {
    case SIM_VIZ_MARKER: {
      if (message.size() != sizeof(sim_viz_marker)) {
        std::println(stderr, "sim.sock: dropped a marker of the wrong size");
        return;
      }
      const auto marker = read_as<sim_viz_marker>(message);
      scene_.drawings[{client, header.id}] = VizMarker{
          .position = to_float3(marker.position),
          .color = to_float3(marker.color),
          .size = marker.size,
      };
      break;
    }
    case SIM_VIZ_PATH: {
      if (message.size() < sizeof(sim_viz_path)) {
        std::println(stderr, "sim.sock: dropped a path of the wrong size");
        return;
      }
      const auto path = read_as<sim_viz_path>(message);
      const auto points = message.subspan(sizeof(sim_viz_path));
      if (path.point_count > SIM_VIZ_MAX_PATH_POINTS || points.size() != path.point_count * sizeof(Float3)) {
        std::println(stderr, "sim.sock: dropped a path whose size doesn't match its point count");
        return;
      }
      VizPath drawing{.points = std::vector<Float3>(path.point_count),
                      .color = to_float3(path.color),
                      .width = path.width};
      std::memcpy(drawing.points.data(), points.data(), points.size());
      scene_.drawings[{client, header.id}] = std::move(drawing);
      break;
    }
    case SIM_VIZ_REMOVE:
      scene_.drawings.erase({client, header.id});
      break;
    case SIM_VIZ_CLEAR:
      erase_client(scene_.drawings, client);
      break;
    default:
      std::println(stderr, "sim.sock: dropped a message of unknown type {}", type);
      return;
  }
  changed();
}

void SimServer::handle_request(Client& client, std::uint32_t type, std::span<const std::byte> message) {
  const auto header = read_as<sim_header>(message);
  const auto reject = [&](std::string_view why) {
    if (header.id != 0) {
      replies_->post({.client = client.id, .message = make_reply(header.id, SIM_INVALID, why)});
    }
  };
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
                 replies->post({.client = client,
                                .message = make_reply(request_id, to_wire(result.status), result.message)});
               });
}

void SimServer::send_replies(std::span<Client> clients) {
  for (const Reply& reply : replies_->take()) {
    const auto client = std::ranges::find(clients, reply.client, &Client::id);
    if (client == clients.end() || !client->fd) {
      continue;  // It disconnected meanwhile.
    }
    // A client that doesn't read its replies loses them rather than blocking us.
    ::send(client->fd.get(), &reply.message, sizeof reply.message, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
}

void SimServer::remove_drawings_of(std::uint64_t client) {
  const std::lock_guard lock{mutex_};
  if (erase_client(scene_.drawings, client)) {
    changed();
  }
}

void SimServer::changed() { version_.store(++scene_.version, std::memory_order_release); }

}  // namespace sim::ipc
