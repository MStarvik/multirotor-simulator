#pragma once

#include <vulkan/vulkan_raii.hpp>

namespace sim::render {

// Records a layout transition (with its memory dependency) for a single-mip,
// single-layer image.
inline void transition(const vk::raii::CommandBuffer& command_buffer, vk::Image image,
                       vk::ImageAspectFlags aspect, vk::ImageLayout old_layout, vk::ImageLayout new_layout,
                       vk::PipelineStageFlags2 src_stage, vk::AccessFlags2 src_access,
                       vk::PipelineStageFlags2 dst_stage, vk::AccessFlags2 dst_access) {
  const vk::ImageMemoryBarrier2 barrier{
      .srcStageMask = src_stage,
      .srcAccessMask = src_access,
      .dstStageMask = dst_stage,
      .dstAccessMask = dst_access,
      .oldLayout = old_layout,
      .newLayout = new_layout,
      .srcQueueFamilyIndex = vk::QueueFamilyIgnored,
      .dstQueueFamilyIndex = vk::QueueFamilyIgnored,
      .image = image,
      .subresourceRange = {.aspectMask = aspect,
                           .baseMipLevel = 0,
                           .levelCount = 1,
                           .baseArrayLayer = 0,
                           .layerCount = 1},
  };
  command_buffer.pipelineBarrier2({.imageMemoryBarrierCount = 1, .pImageMemoryBarriers = &barrier});
}

}  // namespace sim::render
