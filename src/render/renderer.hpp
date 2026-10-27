#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include <vulkan/vulkan_raii.hpp>

#include "render/memory.hpp"
#include "render/scene.hpp"
#include "render/scene_pass.hpp"
#include "render/swapchain.hpp"

namespace sim::render {

class VulkanContext;

// Records, submits and presents frames, keeping up to frames_in_flight frames
// queued on the GPU. Optionally multisampled, and measures GPU time per frame.
class Renderer {
 public:
  static constexpr std::uint32_t frames_in_flight = 2;

  // FrameView::items refer to the meshes by index.
  // msaa_samples is rounded down to a sample count the device supports; 1 disables
  // multisampling.
  Renderer(const VulkanContext& context, vk::Extent2D extent, std::uint32_t msaa_samples,
           std::span<const Mesh> meshes);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  // Recreates the swapchain and the scene's attachments at the given size.
  void resize(vk::Extent2D extent);
  // Replaces the meshes, waiting for the GPU to finish with the old ones. For changes
  // that are rare, such as vehicles coming or going.
  void set_meshes(std::span<const Mesh> meshes);

  // The size of the images being rendered, which may differ slightly from the
  // requested size if the surface constrains it.
  [[nodiscard]] vk::Extent2D extent() const { return swapchain_.extent(); }
  [[nodiscard]] std::uint32_t msaa_samples() const { return scene_pass_.msaa_samples(); }

  // The average GPU time per frame since the last call, from timestamp queries. Empty
  // if no frame finished in between or the queue doesn't support timestamps.
  [[nodiscard]] std::optional<double> take_average_gpu_time_ms();

  void draw_frame(const FrameView& view);

 private:
  // Resources reused by every frames_in_flight-th frame.
  struct Frame {
    vk::raii::CommandPool command_pool;
    vk::raii::CommandBuffer command_buffer;
    vk::raii::Semaphore image_acquired;
    Buffer world_vertices{};  // Grows as needed; see FrameView::world_vertices.
    vk::DeviceSize world_vertices_capacity = 0;
    bool timestamps_written = false;  // The frame's timestamp queries hold unread results.
  };

  void collect_gpu_time(Frame& frame, std::uint32_t first_query);
  void upload_world_vertices(Frame& frame, std::span<const Vertex> vertices) const;
  void record(const Frame& frame, std::uint32_t image_index, std::uint32_t first_query,
              const FrameView& view) const;

  const VulkanContext& context_;
  Swapchain swapchain_;
  // Built for the swapchain's format, which in practice stays the same on recreation.
  ScenePass scene_pass_;
  std::vector<Frame> frames_;
  // Frame n signals value n + 1 when its GPU work completes.
  vk::raii::Semaphore frame_timeline_ = nullptr;
  std::uint64_t frame_number_ = 0;

  // Two timestamps per frame in flight: start and end of its commands.
  vk::raii::QueryPool timestamps_ = nullptr;
  double timestamp_period_ns_ = 0.0;
  double gpu_time_sum_ms_ = 0.0;
  int gpu_time_count_ = 0;
};

}  // namespace sim::render
