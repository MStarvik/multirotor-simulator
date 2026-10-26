#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <vulkan/vulkan_raii.hpp>

#include "render/memory.hpp"
#include "render/scene.hpp"

namespace sim::render {

class VulkanContext;

// Draws lit, vertex-colored meshes with depth testing. The meshes are uploaded once
// at creation; each frame draws any of them with its own transform.
class ScenePass {
 public:
  ScenePass(const VulkanContext& context, vk::Format color_format, vk::Format depth_format,
            vk::SampleCountFlagBits samples, std::span<const Mesh> meshes);

  // Must be called inside dynamic rendering with color and depth attachments of the
  // formats the pass was created with. view.world_vertices must already be in
  // world_vertex_buffer.
  void record(const vk::raii::CommandBuffer& command_buffer, vk::Extent2D extent, const FrameView& view,
              vk::Buffer world_vertex_buffer) const;

 private:
  struct MeshRange {
    std::uint32_t first_vertex;
    std::uint32_t vertex_count;
  };

  vk::raii::PipelineLayout layout_ = nullptr;
  vk::raii::Pipeline pipeline_ = nullptr;
  Buffer vertex_buffer_;
  std::vector<MeshRange> meshes_;
};

}  // namespace sim::render
