#include "ipc/socket_directory.hpp"

#include <cerrno>
#include <format>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>

namespace sim::ipc {
namespace {

// Creates the directory if it doesn't exist, private to the user. Returns whether it
// was created. An existing directory's permissions are the user's choice.
bool create_private_directory(const std::filesystem::path& directory) {
  if (std::filesystem::is_directory(directory)) {
    return false;
  }
  if (directory.has_parent_path()) {
    create_private_directory(directory.parent_path());
  }
  if (!std::filesystem::create_directory(directory)) {
    return false;
  }
  std::filesystem::permissions(directory, std::filesystem::perms::owner_all);
  return true;
}

}  // namespace

SocketDirectory::SocketDirectory(std::filesystem::path directory) : directory_{std::move(directory)} {
  created_directory_ = create_private_directory(directory_);

  // An flock() on the directory is released however the process ends, so unlike a
  // lock file it is never left stale.
  lock_ = UniqueFd{::open(directory_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)};
  if (!lock_) {
    throw std::system_error{errno, std::generic_category(), std::format("open {}", directory_.native())};
  }
  if (::flock(lock_.get(), LOCK_EX | LOCK_NB) != 0) {
    if (errno == EWOULDBLOCK) {
      throw std::runtime_error{
          std::format("{} is used by another simulator; give this one another --instance", directory_.native())};
    }
    throw std::system_error{errno, std::generic_category(), std::format("flock {}", directory_.native())};
  }

  created_vehicles_directory_ = create_private_directory(directory_ / "vehicles");
}

SocketDirectory::~SocketDirectory() {
  // Only if empty: remove() fails otherwise, which is fine.
  std::error_code ignored;
  if (created_vehicles_directory_) {
    std::filesystem::remove(directory_ / "vehicles", ignored);
  }
  if (created_directory_) {
    std::filesystem::remove(directory_, ignored);
  }
}

std::filesystem::path SocketDirectory::vehicle_socket(std::string_view vehicle) const {
  return directory_ / "vehicles" / std::format("{}.sock", vehicle);
}

}  // namespace sim::ipc
