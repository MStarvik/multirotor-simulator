// Checks the simulator socket: that drawings belong to the connection that sent
// them, that requests are applied and answered, and that subscribers are kept up to
// date without a slow one holding anything up.
//
// Usage: sim-server-test <directory for the socket>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <print>
#include <source_location>
#include <stdexcept>
#include <string_view>
#include <thread>

#include <poll.h>
#include <sys/socket.h>

#include <multirotor_sim/sim.h>

#include "ipc/sim_client.hpp"
#include "ipc/sim_server.hpp"
#include "ipc/unix_socket.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

namespace {

using sim::ipc::connect_to;
using sim::ipc::UniqueFd;

int failures = 0;

void check(bool condition, const char* what, std::source_location where = std::source_location::current()) {
  if (!condition) {
    std::println(stderr, "{}:{}: check failed: {}", where.file_name(), where.line(), what);
    ++failures;
  }
}

// Leaves the motors off; the vehicles just sit on the ground.
class IdleController final : public sim::runtime::FlightController {
 public:
  void reset() override {}
  void set_pilot_input(const sim::runtime::PilotInput&) override {}
  [[nodiscard]] std::optional<sim::MotorCommands> update(const sim::runtime::ControllerInput&) override {
    return sim::MotorCommands{};
  }
};

template <typename Message>
void send_message(const UniqueFd& client, const Message& message) {
  ::send(client.get(), &message, sizeof message, MSG_NOSIGNAL);
}

sim_header header(std::uint32_t type, std::uint32_t id) {
  return {.type = type, .version = SIM_PROTOCOL_VERSION, .id = id, .reserved = 0};
}

void send_marker(const UniqueFd& client, std::uint32_t id) {
  send_message(client, sim_viz_marker{
                           .header = header(SIM_VIZ_MARKER, id),
                           .position = {0.0f, 0.0f, -1.0f},
                           .color = {1.0f, 1.0f, 1.0f},
                           .size = 0.1f,
                           .reserved = 0.0f,
                       });
}

// The server handles messages on its own thread; waits until it has caught up.
std::size_t drawings_after_change(const sim::ipc::SimServer& server, std::uint64_t& version) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (server.drawings_version() == version && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  const sim::ipc::VizScene scene = server.drawings();
  version = scene.version;
  return scene.drawings.size();
}

// The next reply, or an empty one with status -1 if none comes in time.
sim_ctl_reply receive_reply(const UniqueFd& client) {
  sim_ctl_reply reply{};
  reply.status = -1;
  pollfd entry{.fd = client.get(), .events = POLLIN, .revents = 0};
  if (::poll(&entry, 1, 2000) == 1) {
    ::recv(client.get(), &reply, sizeof reply, 0);
  }
  return reply;
}

void test_drawings(const sim::ipc::SimServer& server, const std::filesystem::path& socket) {
  std::uint64_t version = server.drawings_version();
  const UniqueFd first = connect_to(socket);
  UniqueFd second = connect_to(socket);

  send_marker(first, 1);
  check(drawings_after_change(server, version) == 1, "a marker is drawn");
  send_marker(second, 1);
  check(drawings_after_change(server, version) == 2, "clients with the same id don't replace each other");
  send_marker(first, 1);
  check(drawings_after_change(server, version) == 2, "a client replaces its own drawing");
  send_message(first, header(SIM_VIZ_CLEAR, 0));
  check(drawings_after_change(server, version) == 1, "a client only clears its own drawings");
  second.reset();
  check(drawings_after_change(server, version) == 0, "a client's drawings go when it disconnects");
}

void test_requests(const std::filesystem::path& socket) {
  const UniqueFd client = connect_to(socket);

  send_message(client, sim_ctl_pause{.header = header(SIM_CTL_PAUSE, 7), .paused = 1, .reserved = 0});
  const sim_ctl_reply paused = receive_reply(client);
  check(paused.header.type == SIM_CTL_REPLY && paused.header.id == 7 && paused.status == SIM_OK,
        "pausing is answered with the request's id");

  sim_ctl_pilot_input input{.header = header(SIM_CTL_PILOT_INPUT, 8), .vehicle = {}, .sticks = {}};
  std::memcpy(input.vehicle, "nobody", 6);
  send_message(client, input);
  const sim_ctl_reply unknown = receive_reply(client);
  check(unknown.header.id == 8 && unknown.status == SIM_NOT_FOUND, "input for an unknown vehicle is refused");
  check(std::string_view{unknown.message}.contains("nobody"), "the refusal names the vehicle");

  std::memcpy(input.vehicle, "alpha", 6);
  input.header.id = 9;
  send_message(client, input);
  check(receive_reply(client).status == SIM_OK, "input for a vehicle is accepted");

  send_message(client, sim_ctl_set_speed{.header = header(SIM_CTL_SET_SPEED, 10), .speed = 2.0});
  check(receive_reply(client).status == SIM_UNSUPPORTED, "an unpaced simulator can't change speed");

  send_message(client, header(SIM_CTL_RESET, 11));
  send_message(client, header(SIM_CTL_RESET, 0));  // Asks for no reply.
  send_message(client, header(SIM_CTL_RESET, 12));
  check(receive_reply(client).header.id == 11 && receive_reply(client).header.id == 12,
        "a request with id 0 isn't answered");
}

// Receives on the client until the condition holds, or two seconds pass.
template <typename Condition>
bool receive_until(sim::ipc::SimClient& client, Condition condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (std::chrono::steady_clock::now() < deadline) {
    client.request_state();
    if (!client.receive()) {
      return false;
    }
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return false;
}

void test_subscriber(const std::filesystem::path& socket) {
  sim::ipc::SimClient viewer{socket};
  check(receive_until(viewer, [&] { return viewer.snapshot().vehicles.size() == 1; }),
        "a subscriber gets the vehicles and their state");
  check(viewer.snapshot().roster->vehicles.at(0).name == "alpha", "the vehicles have their names");
  check(viewer.snapshot().roster->vehicles.at(0).model.rotors.size() == 4, "the vehicles have their rotors");

  UniqueFd drawer = connect_to(socket);
  send_marker(drawer, 1);
  send_marker(drawer, 2);
  check(receive_until(viewer, [&] { return viewer.drawings().drawings.size() == 2; }),
        "a subscriber gets other clients' drawings");
  drawer.reset();
  check(receive_until(viewer, [&] { return viewer.drawings().drawings.empty(); }),
        "a subscriber sees drawings go when their client disconnects");

  // A subscriber that never reads fills its socket. The others must carry on.
  sim::ipc::SimClient stuck{socket};
  const UniqueFd busy = connect_to(socket);
  for (std::uint32_t i = 0; i < 2000; ++i) {
    send_marker(busy, i);
  }
  const UniqueFd requester = connect_to(socket);
  send_message(requester, header(SIM_CTL_RESET, 20));
  check(receive_reply(requester).header.id == 20, "a stuck subscriber doesn't hold up requests");
  check(receive_until(viewer, [&] { return viewer.drawings().drawings.size() == 2000; }),
        "a stuck subscriber doesn't hold up the others");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::println(stderr, "usage: sim-server-test <directory>");
    return EXIT_FAILURE;
  }
  const std::filesystem::path socket = std::filesystem::path{argv[1]} / "sim.sock";

  sim::runtime::SimRunner runner{0.001, 1, [](const auto&) { return std::make_unique<IdleController>(); },
                                 sim::runtime::AsFastAsPossible{}};
  runner.add_vehicle({.name = "alpha", .model = sim::make_multirotor({}), .initial_state = {}});
  const sim::ipc::SimServer server{socket, runner, {}};
  {
    const std::jthread simulation{[&runner](std::stop_token stop) { runner.run(stop); }};
    test_drawings(server, socket);
    test_subscriber(socket);
    test_requests(socket);
  }
  check(runner.latest().paused, "the simulation was paused");

  if (failures == 0) {
    std::println("All simulator socket checks passed");
  }
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
