#include "render/offscreen_renderer.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <print>
#include <utility>
#include <stdexcept>

#include "render/barrier.hpp"
#include "render/vulkan_context.hpp"

namespace sim::render {
namespace {

constexpr std::uint64_t no_timeout = std::numeric_limits<std::uint64_t>::max();
constexpr vk::DeviceSize bytes_per_pixel = 4;

}  // namespace

OffscreenRenderer::OffscreenRenderer(const VulkanContext& context, vk::Extent2D extent,
                                     std::uint32_t msaa_samples, std::span<const Mesh> meshes)
    : context_{context},
      extent_{extent},
      target_{create_attachment_image(
          context, format, extent, vk::SampleCountFlagBits::e1,
          vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc,
          vk::ImageAspectFlagBits::eColor)},
      scene_pass_{context, format, extent, msaa_samples, meshes} {
  const auto& device = context_.device();
  command_pool_ = device.createCommandPool({
      .flags = vk::CommandPoolCreateFlagBits::eTransient,
      .queueFamilyIndex = context_.queue_family(),
  });
  command_buffer_ = std::move(vk::raii::CommandBuffers{device,
                                                       {
                                                           .commandPool = *command_pool_,
                                                           .level = vk::CommandBufferLevel::ePrimary,
                                                           .commandBufferCount = 1,
                                                       }}
                                  .front());
  frame_done_ = device.createFence({});

  const vk::DeviceSize frame_size = vk::DeviceSize{extent.width} * extent.height * bytes_per_pixel;
  readback_ = create_readback_buffer(context_, frame_size);
  pixels_.resize(frame_size);
}

OffscreenRenderer::~OffscreenRenderer() {
  try {
    context_.device().waitIdle();
  } catch (const vk::SystemError& error) {
    std::println(stderr, "error: waiting for the GPU at shutdown failed: {}", error.what());
  }
}

std::span<const std::byte> OffscreenRenderer::render(const FrameView& view) {
  // The previous frame has finished (render() waits for it), so all buffers are free.
  upload_world_vertices(view.world_vertices);
  command_pool_.reset();
  record(view);

  const vk::CommandBufferSubmitInfo command_buffer_info{.commandBuffer = *command_buffer_};
  context_.queue().submit2(
      vk::SubmitInfo2{.commandBufferInfoCount = 1, .pCommandBufferInfos = &command_buffer_info},
      *frame_done_);
  if (context_.device().waitForFences(*frame_done_, vk::True, no_timeout) != vk::Result::eSuccess) {
    throw std::runtime_error{"timed out waiting for a frame to render"};
  }
  context_.device().resetFences(*frame_done_);

  read_host_buffer(readback_, pixels_);
  return pixels_;
}

void OffscreenRenderer::upload_world_vertices(std::span<const Vertex> vertices) {
  const auto bytes = std::as_bytes(vertices);
  if (bytes.empty()) {
    return;
  }
  if (bytes.size() > world_vertices_capacity_) {
    world_vertices_capacity_ = std::max<vk::DeviceSize>(bytes.size(), 2 * world_vertices_capacity_);
    world_vertices_ =
        create_host_buffer(context_, vk::BufferUsageFlagBits::eVertexBuffer, world_vertices_capacity_);
  }
  write_host_buffer(world_vertices_, bytes);
}

void OffscreenRenderer::record(const FrameView& view) const {
  const vk::raii::CommandBuffer& cmd = command_buffer_;
  const vk::Image image = *target_.image;
  cmd.begin({.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  // The previous frame was fully read back before this one started, so only the
  // layout needs changing.
  transition(cmd, image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eUndefined,
             vk::ImageLayout::eColorAttachmentOptimal, vk::PipelineStageFlagBits2::eNone,
             vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput,
             vk::AccessFlagBits2::eColorAttachmentWrite);
  scene_pass_.record(cmd, *target_.view, view, *world_vertices_.buffer);

  // Copy the frame to the readback buffer, and make the copy visible to the CPU.
  transition(cmd, image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eColorAttachmentOptimal,
             vk::ImageLayout::eTransferSrcOptimal, vk::PipelineStageFlagBits2::eColorAttachmentOutput,
             vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eCopy,
             vk::AccessFlagBits2::eTransferRead);
  cmd.copyImageToBuffer(image, vk::ImageLayout::eTransferSrcOptimal, *readback_.buffer,
                        vk::BufferImageCopy{
                            .bufferOffset = 0,
                            .bufferRowLength = 0,  // Tightly packed.
                            .bufferImageHeight = 0,
                            .imageSubresource = {.aspectMask = vk::ImageAspectFlagBits::eColor,
                                                 .mipLevel = 0,
                                                 .baseArrayLayer = 0,
                                                 .layerCount = 1},
                            .imageOffset = {.x = 0, .y = 0, .z = 0},
                            .imageExtent = {.width = extent_.width, .height = extent_.height, .depth = 1},
                        });
  const vk::BufferMemoryBarrier2 to_host{
      .srcStageMask = vk::PipelineStageFlagBits2::eCopy,
      .srcAccessMask = vk::AccessFlagBits2::eTransferWrite,
      .dstStageMask = vk::PipelineStageFlagBits2::eHost,
      .dstAccessMask = vk::AccessFlagBits2::eHostRead,
      .srcQueueFamilyIndex = vk::QueueFamilyIgnored,
      .dstQueueFamilyIndex = vk::QueueFamilyIgnored,
      .buffer = *readback_.buffer,
      .offset = 0,
      .size = vk::WholeSize,
  };
  cmd.pipelineBarrier2({.bufferMemoryBarrierCount = 1, .pBufferMemoryBarriers = &to_host});

  cmd.end();
}

}  // namespace sim::render
