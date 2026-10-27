// Checks that drawings belong to the connection that sent them.
//
// Usage: viz-test <directory for the socket>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <print>
#include <source_location>
#include <stdexcept>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>

#include <multirotor_sim/viz.h>

#include "ipc/unix_socket.hpp"
#include "ipc/viz_server.hpp"

namespace {

using sim::ipc::UniqueFd;

int failures = 0;

void check(bool condition, const char* what, std::source_location where = std::source_location::current()) {
  if (!condition) {
    std::println(stderr, "{}:{}: check failed: {}", where.file_name(), where.line(), what);
    ++failures;
  }
}

UniqueFd connect_to(const std::filesystem::path& path) {
  UniqueFd fd{::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0)};
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof address.sun_path - 1);
  if (!fd || ::connect(fd.get(), reinterpret_cast<const sockaddr*>(&address), sizeof address) != 0) {
    throw std::runtime_error{"could not connect to the viz socket"};
  }
  return fd;
}

void send_marker(const UniqueFd& client, std::uint32_t id) {
  const sim_viz_marker marker{
      .header = {.type = SIM_VIZ_MARKER, .version = SIM_VIZ_PROTOCOL_VERSION, .id = id, .reserved = 0},
      .position = {0.0f, 0.0f, -1.0f},
      .color = {1.0f, 1.0f, 1.0f},
      .size = 0.1f,
      .reserved = 0.0f,
  };
  ::send(client.get(), &marker, sizeof marker, MSG_NOSIGNAL);
}

void send_clear(const UniqueFd& client) {
  const sim_viz_header clear{.type = SIM_VIZ_CLEAR, .version = SIM_VIZ_PROTOCOL_VERSION, .id = 0, .reserved = 0};
  ::send(client.get(), &clear, sizeof clear, MSG_NOSIGNAL);
}

// The server handles messages on its own thread; waits until it has caught up.
std::size_t drawings_after_change(const sim::ipc::VizServer& server, std::uint64_t& version) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (server.version() == version && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  const sim::ipc::VizScene scene = server.scene();
  version = scene.version;
  return scene.drawings.size();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::println(stderr, "usage: viz-test <directory>");
    return EXIT_FAILURE;
  }
  const sim::ipc::VizServer server{std::filesystem::path{argv[1]} / "viz.sock"};
  std::uint64_t version = server.version();

  const UniqueFd first = connect_to(std::filesystem::path{argv[1]} / "viz.sock");
  UniqueFd second = connect_to(std::filesystem::path{argv[1]} / "viz.sock");

  send_marker(first, 1);
  check(drawings_after_change(server, version) == 1, "a marker is drawn");
  send_marker(second, 1);
  check(drawings_after_change(server, version) == 2, "clients with the same id don't replace each other");
  send_marker(first, 1);
  check(drawings_after_change(server, version) == 2, "a client replaces its own drawing");
  send_clear(first);
  check(drawings_after_change(server, version) == 1, "a client only clears its own drawings");
  second.reset();
  check(drawings_after_change(server, version) == 0, "a client's drawings go when it disconnects");

  if (failures == 0) {
    std::println("All viz checks passed");
  }
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
