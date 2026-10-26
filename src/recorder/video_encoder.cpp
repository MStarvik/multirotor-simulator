#include "recorder/video_encoder.hpp"

#include <cerrno>
#include <format>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace sim::recorder {
namespace {

[[noreturn]] void throw_errno(const char* what) {
  throw std::system_error{errno, std::generic_category(), what};
}

}  // namespace

VideoEncoder::VideoEncoder(const std::filesystem::path& output, std::uint32_t width, std::uint32_t height,
                           std::uint32_t frame_rate) {
  // Raw RGBA frames in on standard input, VP9 out. CRF 32 with no bitrate cap is
  // VP9's constant-quality mode. The color matrix and tags are BT.709, which is what
  // browsers assume for HD video, so colors come out as rendered.
  const std::vector<std::string> arguments{
      "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
      "-f", "rawvideo", "-pix_fmt", "rgba", "-video_size", std::format("{}x{}", width, height),
      "-framerate", std::to_string(frame_rate), "-i", "pipe:0",
      "-vf", "scale=out_color_matrix=bt709:out_range=tv", "-pix_fmt", "yuv420p",
      "-colorspace", "bt709", "-color_primaries", "bt709", "-color_trc", "bt709",
      "-c:v", "libvpx-vp9", "-crf", "32", "-b:v", "0", "-row-mt", "1", "-deadline", "good", "-cpu-used", "2",
      output.native(),
  };

  int fds[2];
  if (::pipe2(fds, O_CLOEXEC) != 0) {
    throw_errno("pipe2");
  }
  const int read_end = fds[0];
  pipe_ = fds[1];

  // The child gets the read end as standard input; dup2 clears close-on-exec.
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, read_end, STDIN_FILENO);

  std::vector<char*> argv;
  for (const std::string& argument : arguments) {
    argv.push_back(const_cast<char*>(argument.c_str()));
  }
  argv.push_back(nullptr);

  const int error = ::posix_spawnp(&ffmpeg_, "ffmpeg", &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  ::close(read_end);
  if (error != 0) {
    ::close(pipe_);
    throw std::system_error{error, std::generic_category(), "could not start ffmpeg"};
  }
}

VideoEncoder::~VideoEncoder() {
  if (pipe_ >= 0) {
    ::close(pipe_);
  }
  if (ffmpeg_ > 0) {
    int status = 0;
    ::waitpid(ffmpeg_, &status, 0);
  }
}

void VideoEncoder::write(std::span<const std::byte> frame) {
  while (!frame.empty()) {
    const ssize_t written = ::write(pipe_, frame.data(), frame.size());
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::system_error{errno, std::generic_category(), "ffmpeg stopped accepting frames"};
    }
    frame = frame.subspan(static_cast<std::size_t>(written));
  }
}

void VideoEncoder::finish() {
  ::close(std::exchange(pipe_, -1));  // End of input: ffmpeg finishes the file.
  int status = 0;
  if (::waitpid(std::exchange(ffmpeg_, -1), &status, 0) < 0) {
    throw_errno("waitpid");
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error{"ffmpeg failed to encode the video"};
  }
}

}  // namespace sim::recorder
