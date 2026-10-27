#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <vulkan/vulkan_raii.hpp>

#include "render/memory.hpp"
#include "render/scene.hpp"

namespace sim::render {

class VulkanContext;

// Draws lit, vertex-colored meshes with depth testing into an output image, with
// optional multisampling. The meshes are uploaded at creation, and again when
// replaced; each frame draws any of them with its own transform. Owns the depth buffer and the multisampled
// color image, so callers only provide the image to end up in.
class ScenePass {
 public:
  // msaa_samples is rounded down to a sample count the device supports; 1 disables
  // multisampling.
  ScenePass(const VulkanContext& context, vk::Format color_format, vk::Extent2D extent,
            std::uint32_t msaa_samples, std::span<const Mesh> meshes);

  // Recreates the attachments for a new output size. The GPU must not be using them.
  void resize(vk::Extent2D extent);
  // Replaces the meshes that frames refer to by index. The GPU must not be using them.
  void set_meshes(std::span<const Mesh> meshes);

  [[nodiscard]] std::uint32_t msaa_samples() const { return static_cast<std::uint32_t>(samples_); }

  // Draws the frame into `output`: an image view of the pass's color format and
  // extent, already in eColorAttachmentOptimal, and left there with the writes made in
  // eColorAttachmentOutput. view.world_vertices must already be in world_vertex_buffer.
  void record(const vk::raii::CommandBuffer& command_buffer, vk::ImageView output, const FrameView& view,
              vk::Buffer world_vertex_buffer) const;

 private:
  struct MeshRange {
    std::uint32_t first_vertex;
    std::uint32_t vertex_count;
  };

  [[nodiscard]] bool multisampled() const { return samples_ != vk::SampleCountFlagBits::e1; }
  void draw(const vk::raii::CommandBuffer& command_buffer, const FrameView& view,
            vk::Buffer world_vertex_buffer) const;

  const VulkanContext& context_;
  vk::Format color_format_;
  vk::Format depth_format_;
  vk::SampleCountFlagBits samples_;
  vk::Extent2D extent_;
  // Cleared every frame, so one of each is enough: the barriers at the start of each
  // frame order their use after the previous frame's on the same queue.
  Image color_image_;  // Multisampled; resolved into the output. Unused without MSAA.
  Image depth_image_;

  vk::raii::PipelineLayout layout_ = nullptr;
  vk::raii::Pipeline pipeline_ = nullptr;
  Buffer vertex_buffer_;
  std::vector<MeshRange> meshes_;
};

}  // namespace sim::render
