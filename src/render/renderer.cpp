#include "render/renderer.hpp"

#include <algorithm>
#include <limits>
#include <span>
#include <print>
#include <stdexcept>

#include "render/barrier.hpp"
#include "render/vulkan_context.hpp"

namespace sim::render {
namespace {

constexpr std::uint64_t no_timeout = std::numeric_limits<std::uint64_t>::max();

}  // namespace

Renderer::Renderer(const VulkanContext& context, vk::Extent2D extent, std::uint32_t msaa_samples,
                   std::span<const Mesh> meshes)
    : context_{context},
      swapchain_{context, extent},
      scene_pass_{context, swapchain_.format(), swapchain_.extent(), msaa_samples, meshes} {
  const auto& device = context_.device();

  // GPU timing, if the queue supports timestamps.
  const auto& physical_device = context_.physical_device();
  if (physical_device.getQueueFamilyProperties()[context_.queue_family()].timestampValidBits > 0) {
    timestamps_ = device.createQueryPool({
        .queryType = vk::QueryType::eTimestamp,
        .queryCount = 2 * frames_in_flight,
    });
    timestamp_period_ns_ = static_cast<double>(physical_device.getProperties().limits.timestampPeriod);
  }

  for (std::uint32_t i = 0; i < frames_in_flight; ++i) {
    vk::raii::CommandPool pool = device.createCommandPool({
        .flags = vk::CommandPoolCreateFlagBits::eTransient,
        .queueFamilyIndex = context_.queue_family(),
    });
    vk::raii::CommandBuffers buffers{device, {
        .commandPool = *pool,
        .level = vk::CommandBufferLevel::ePrimary,
        .commandBufferCount = 1,
    }};
    frames_.push_back(Frame{
        .command_pool = std::move(pool),
        .command_buffer = std::move(buffers.front()),
        .image_acquired = device.createSemaphore({}),
    });
  }

  const vk::SemaphoreTypeCreateInfo timeline_info{
      .semaphoreType = vk::SemaphoreType::eTimeline,
      .initialValue = 0,
  };
  frame_timeline_ = device.createSemaphore({.pNext = &timeline_info});
}

Renderer::~Renderer() {
  try {
    context_.device().waitIdle();
  } catch (const vk::SystemError& error) {
    std::println(stderr, "error: waiting for the GPU at shutdown failed: {}", error.what());
  }
}

void Renderer::resize(vk::Extent2D extent) {
  // Simple and safe: let all in-flight work finish before replacing the images.
  context_.device().waitIdle();
  swapchain_.recreate(extent);
  scene_pass_.resize(swapchain_.extent());
}

std::optional<double> Renderer::take_average_gpu_time_ms() {
  if (gpu_time_count_ == 0) {
    return std::nullopt;
  }
  const double average = gpu_time_sum_ms_ / gpu_time_count_;
  gpu_time_sum_ms_ = 0.0;
  gpu_time_count_ = 0;
  return average;
}

void Renderer::collect_gpu_time(Frame& frame, std::uint32_t first_query) {
  if (!frame.timestamps_written) {
    return;
  }
  frame.timestamps_written = false;
  // The frame has finished on the GPU, so this doesn't wait.
  const auto [result, ticks] = timestamps_.getResults<std::uint64_t>(
      first_query, 2, 2 * sizeof(std::uint64_t), sizeof(std::uint64_t), vk::QueryResultFlagBits::e64);
  if (result == vk::Result::eSuccess) {
    gpu_time_sum_ms_ += static_cast<double>(ticks[1] - ticks[0]) * timestamp_period_ns_ * 1e-6;
    ++gpu_time_count_;
  }
}

void Renderer::draw_frame(const FrameView& view) {
  const auto& device = context_.device();
  const auto slot = static_cast<std::uint32_t>(frame_number_ % frames_in_flight);
  Frame& frame = frames_[slot];
  const std::uint32_t first_query = 2 * slot;

  // Wait until the GPU is done with the frame that last used these resources.
  if (frame_number_ >= frames_in_flight) {
    const std::uint64_t value = frame_number_ - frames_in_flight + 1;
    if (device.waitSemaphores({.semaphoreCount = 1, .pSemaphores = &*frame_timeline_, .pValues = &value},
                              no_timeout) != vk::Result::eSuccess) {
      throw std::runtime_error{"timed out waiting for a frame to finish"};
    }
  }
  collect_gpu_time(frame, first_query);

  const auto [acquire_result, image_index] =
      swapchain_.handle().acquireNextImage(no_timeout, *frame.image_acquired);
  if (acquire_result == vk::Result::eErrorOutOfDateKHR) {
    resize(swapchain_.extent());
    return;
  }

  upload_world_vertices(frame, view.world_vertices);
  frame.command_pool.reset();
  record(frame, image_index, first_query, view);
  frame.timestamps_written = *timestamps_ != nullptr;

  const vk::Semaphore present_semaphore = swapchain_.present_semaphore(image_index);
  const vk::SemaphoreSubmitInfo wait_info{
      .semaphore = *frame.image_acquired,
      .stageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput,
  };
  const std::array signal_infos{
      vk::SemaphoreSubmitInfo{
          .semaphore = present_semaphore,
          .stageMask = vk::PipelineStageFlagBits2::eAllCommands,
      },
      vk::SemaphoreSubmitInfo{
          .semaphore = *frame_timeline_,
          .value = frame_number_ + 1,
          .stageMask = vk::PipelineStageFlagBits2::eAllCommands,
      },
  };
  const vk::CommandBufferSubmitInfo command_buffer_info{.commandBuffer = *frame.command_buffer};
  context_.queue().submit2(vk::SubmitInfo2{
      .waitSemaphoreInfoCount = 1,
      .pWaitSemaphoreInfos = &wait_info,
      .commandBufferInfoCount = 1,
      .pCommandBufferInfos = &command_buffer_info,
      .signalSemaphoreInfoCount = static_cast<std::uint32_t>(signal_infos.size()),
      .pSignalSemaphoreInfos = signal_infos.data(),
  });
  ++frame_number_;

  const vk::SwapchainKHR swapchain = *swapchain_.handle();
  const vk::Result present_result = context_.queue().presentKHR({
      .waitSemaphoreCount = 1,
      .pWaitSemaphores = &present_semaphore,
      .swapchainCount = 1,
      .pSwapchains = &swapchain,
      .pImageIndices = &image_index,
  });
  if (present_result == vk::Result::eErrorOutOfDateKHR ||
      present_result == vk::Result::eSuboptimalKHR || acquire_result == vk::Result::eSuboptimalKHR) {
    resize(swapchain_.extent());
  }
}

void Renderer::upload_world_vertices(Frame& frame, std::span<const Vertex> vertices) const {
  // Safe: the wait at the start of draw_frame() means the GPU is done with this frame.
  const auto bytes = std::as_bytes(vertices);
  if (bytes.empty()) {
    return;
  }
  if (bytes.size() > frame.world_vertices_capacity) {
    // Grow geometrically so a slowly growing path doesn't reallocate every frame.
    frame.world_vertices_capacity = std::max<vk::DeviceSize>(bytes.size(), 2 * frame.world_vertices_capacity);
    frame.world_vertices = create_host_buffer(context_, vk::BufferUsageFlagBits::eVertexBuffer,
                                              frame.world_vertices_capacity);
  }
  write_host_buffer(frame.world_vertices, bytes);
}

void Renderer::record(const Frame& frame, std::uint32_t image_index, std::uint32_t first_query,
                      const FrameView& view) const {
  const vk::raii::CommandBuffer& cmd = frame.command_buffer;
  const vk::Image image = swapchain_.image(image_index);

  cmd.begin({.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  if (*timestamps_ != nullptr) {
    cmd.resetQueryPool(*timestamps_, first_query, 2);
    cmd.writeTimestamp2(vk::PipelineStageFlagBits2::eTopOfPipe, *timestamps_, first_query);
  }

  // The previous contents are discarded, so the old layout can be undefined. The
  // source stage matches the acquire semaphore's wait stage to order after it.
  transition(cmd, image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eUndefined,
             vk::ImageLayout::eColorAttachmentOptimal,
             vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eNone,
             vk::PipelineStageFlagBits2::eColorAttachmentOutput,
             vk::AccessFlagBits2::eColorAttachmentWrite);
  scene_pass_.record(cmd, swapchain_.image_view(image_index), view, *frame.world_vertices.buffer);

  transition(cmd, image, vk::ImageAspectFlagBits::eColor, vk::ImageLayout::eColorAttachmentOptimal,
             vk::ImageLayout::ePresentSrcKHR,
             vk::PipelineStageFlagBits2::eColorAttachmentOutput,
             vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eNone,
             vk::AccessFlagBits2::eNone);

  if (*timestamps_ != nullptr) {
    cmd.writeTimestamp2(vk::PipelineStageFlagBits2::eBottomOfPipe, *timestamps_, first_query + 1);
  }
  cmd.end();
}

}  // namespace sim::render
