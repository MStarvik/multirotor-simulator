#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <vulkan/vulkan_raii.hpp>

#include "render/memory.hpp"
#include "render/scene.hpp"
#include "render/scene_pass.hpp"

namespace sim::render {

class VulkanContext;

// Renders frames into an image and reads them back to the CPU, without a window or
// display. One frame at a time: simple, and fast enough for recording video.
class OffscreenRenderer {
 public:
  static constexpr vk::Format format = vk::Format::eR8G8B8A8Srgb;

  // FrameView::items refer to the meshes by index.
  OffscreenRenderer(const VulkanContext& context, vk::Extent2D extent, std::uint32_t msaa_samples,
                    std::span<const Mesh> meshes);
  ~OffscreenRenderer();

  OffscreenRenderer(const OffscreenRenderer&) = delete;
  OffscreenRenderer& operator=(const OffscreenRenderer&) = delete;

  // Replaces the meshes. Frames are rendered one at a time, so the GPU is done with
  // the old ones.
  void set_meshes(std::span<const Mesh> meshes) { scene_pass_.set_meshes(meshes); }

  [[nodiscard]] vk::Extent2D extent() const { return extent_; }
  [[nodiscard]] std::uint32_t msaa_samples() const { return scene_pass_.msaa_samples(); }

  // Renders a frame and returns its pixels: 8-bit sRGB RGBA, rows from top to bottom.
  // The data stays valid until the next call.
  [[nodiscard]] std::span<const std::byte> render(const FrameView& view);

 private:
  void upload_world_vertices(std::span<const Vertex> vertices);
  void record(const FrameView& view) const;

  const VulkanContext& context_;
  vk::Extent2D extent_;
  Image target_;  // The finished frame, copied to readback_.
  ScenePass scene_pass_;

  vk::raii::CommandPool command_pool_ = nullptr;
  vk::raii::CommandBuffer command_buffer_ = nullptr;
  vk::raii::Fence frame_done_ = nullptr;

  Buffer readback_;
  Buffer world_vertices_;
  vk::DeviceSize world_vertices_capacity_ = 0;
  std::vector<std::byte> pixels_;
};

}  // namespace sim::render
