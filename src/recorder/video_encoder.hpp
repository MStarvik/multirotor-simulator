#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>

#include <sys/types.h>

namespace sim::recorder {

// Encodes raw frames by piping them to ffmpeg, which must be on the PATH. Writes an
// animated AV1 image for a .avif output, and VP9 video otherwise (use .webm).
class VideoEncoder {
 public:
  VideoEncoder(const std::filesystem::path& output, std::uint32_t width, std::uint32_t height,
               std::uint32_t frame_rate);
  // Abandons the video if finish() wasn't called.
  ~VideoEncoder();

  VideoEncoder(const VideoEncoder&) = delete;
  VideoEncoder& operator=(const VideoEncoder&) = delete;

  // One frame of 8-bit sRGB RGBA, rows from top to bottom.
  void write(std::span<const std::byte> frame);

  // Waits for ffmpeg to finish the file. Throws if encoding failed.
  void finish();

 private:
  int pipe_ = -1;  // ffmpeg's standard input.
  pid_t ffmpeg_ = -1;
};

}  // namespace sim::recorder
