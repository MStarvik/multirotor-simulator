#pragma once

#include <filesystem>
#include <string_view>

#include "ipc/unix_socket.hpp"

namespace sim::ipc {

// The directory that holds one simulator's sockets:
//
//   <directory>/viz.sock              drawings, for the whole simulation
//   <directory>/vehicles/<name>.sock  SITL, one per vehicle
//
// Claims the directory for as long as it exists, so that two simulators can't
// share one, and removes the directories it created again when destroyed. Destroy
// it after the sockets in it.
class SocketDirectory {
 public:
  // Throws if another simulator has claimed the directory.
  explicit SocketDirectory(std::filesystem::path directory);
  ~SocketDirectory();

  SocketDirectory(const SocketDirectory&) = delete;
  SocketDirectory& operator=(const SocketDirectory&) = delete;

  [[nodiscard]] const std::filesystem::path& path() const { return directory_; }
  [[nodiscard]] std::filesystem::path viz_socket() const { return directory_ / "viz.sock"; }
  [[nodiscard]] std::filesystem::path vehicle_socket(std::string_view vehicle) const;

 private:
  std::filesystem::path directory_;
  bool created_directory_ = false;
  bool created_vehicles_directory_ = false;
  UniqueFd lock_;  // The directory itself, locked with flock().
};

}  // namespace sim::ipc
