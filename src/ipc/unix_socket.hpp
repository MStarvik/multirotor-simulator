#pragma once

#include <filesystem>
#include <utility>

namespace sim::ipc {

// Owns a file descriptor.
class UniqueFd {
 public:
  UniqueFd() = default;
  explicit UniqueFd(int fd) : fd_{fd} {}
  ~UniqueFd() { reset(); }

  UniqueFd(UniqueFd&& other) noexcept : fd_{std::exchange(other.fd_, -1)} {}
  UniqueFd& operator=(UniqueFd&& other) noexcept {
    if (this != &other) {
      reset(std::exchange(other.fd_, -1));
    }
    return *this;
  }

  [[nodiscard]] int get() const { return fd_; }
  explicit operator bool() const { return fd_ >= 0; }
  void reset(int fd = -1);

 private:
  int fd_ = -1;
};

// A listening SOCK_SEQPACKET UNIX socket. Creates the parent directory (private to
// the user) if needed, and removes the socket file again when destroyed.
class ListeningSocket {
 public:
  // Throws if the path is too long, or if another process is listening on it.
  explicit ListeningSocket(std::filesystem::path path);
  ~ListeningSocket();

  ListeningSocket(const ListeningSocket&) = delete;
  ListeningSocket& operator=(const ListeningSocket&) = delete;

  [[nodiscard]] int fd() const { return fd_.get(); }
  [[nodiscard]] const std::filesystem::path& path() const { return path_; }

  // Accepts a pending connection, or returns an empty UniqueFd if there is none.
  [[nodiscard]] UniqueFd accept() const;

 private:
  std::filesystem::path path_;
  UniqueFd fd_;
};

// Connects to a SOCK_SEQPACKET UNIX socket. Throws std::system_error if nothing
// listens there.
[[nodiscard]] UniqueFd connect_to(const std::filesystem::path& path);

// Waits up to timeout_ms for fd to become readable. Returns poll's revents, or 0 on
// timeout or interruption.
[[nodiscard]] short wait_readable(int fd, int timeout_ms);

}  // namespace sim::ipc
