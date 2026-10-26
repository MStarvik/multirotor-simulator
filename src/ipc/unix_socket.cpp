#include "ipc/unix_socket.hpp"

#include <cerrno>
#include <cstring>
#include <format>
#include <stdexcept>
#include <system_error>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace sim::ipc {
namespace {

[[noreturn]] void throw_errno(std::string_view what) {
  throw std::system_error{errno, std::generic_category(), std::string{what}};
}

sockaddr_un make_address(const std::filesystem::path& path) {
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  const std::string& native = path.native();
  if (native.size() >= sizeof(address.sun_path)) {
    throw std::invalid_argument{std::format("socket path is too long: {}", native)};
  }
  std::memcpy(address.sun_path, native.c_str(), native.size() + 1);
  return address;
}

// Whether some process accepts connections on the socket at this address.
bool is_in_use(const sockaddr_un& address) {
  const UniqueFd probe{::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0)};
  return probe && ::connect(probe.get(), reinterpret_cast<const sockaddr*>(&address), sizeof address) == 0;
}

}  // namespace

void UniqueFd::reset(int fd) {
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = fd;
}

ListeningSocket::ListeningSocket(std::filesystem::path path) : path_{std::move(path)} {
  const sockaddr_un address = make_address(path_);

  // Only restrict a directory we create; an existing one is the user's choice.
  if (path_.has_parent_path() && std::filesystem::create_directories(path_.parent_path())) {
    std::filesystem::permissions(path_.parent_path(), std::filesystem::perms::owner_all);
  }
  if (is_in_use(address)) {
    throw std::runtime_error{std::format("{} is in use; is another simulator running?", path_.native())};
  }
  std::filesystem::remove(path_);  // A stale socket file from a run that didn't exit cleanly.

  fd_ = UniqueFd{::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0)};
  if (!fd_) {
    throw_errno("socket");
  }
  if (::bind(fd_.get(), reinterpret_cast<const sockaddr*>(&address), sizeof address) != 0) {
    throw_errno(std::format("bind {}", path_.native()));
  }
  if (::listen(fd_.get(), 8) != 0) {
    throw_errno("listen");
  }
}

ListeningSocket::~ListeningSocket() {
  std::error_code ignored;
  std::filesystem::remove(path_, ignored);
}

UniqueFd ListeningSocket::accept() const {
  return UniqueFd{::accept4(fd_.get(), nullptr, nullptr, SOCK_CLOEXEC)};
}

short wait_readable(int fd, int timeout_ms) {
  pollfd entry{.fd = fd, .events = POLLIN, .revents = 0};
  return ::poll(&entry, 1, timeout_ms) > 0 ? entry.revents : 0;
}

}  // namespace sim::ipc
