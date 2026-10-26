#pragma once

#include <cstddef>
#include <span>

#include <vulkan/vulkan_raii.hpp>

// Minimal resource creation. Each resource gets its own memory allocation, which is
// fine for a handful of resources; switch to VMA (Vulkan Memory Allocator) as the
// number grows.

namespace sim::render {

class VulkanContext;

// Members are declared so that the resource is destroyed before its memory.
struct Buffer {
  vk::raii::DeviceMemory memory = nullptr;
  vk::raii::Buffer buffer = nullptr;
};

struct Image {
  vk::raii::DeviceMemory memory = nullptr;
  vk::raii::Image image = nullptr;
  vk::raii::ImageView view = nullptr;
};

// A host-visible buffer of `size` bytes, for data the CPU writes and the GPU reads.
Buffer create_host_buffer(const VulkanContext& context, vk::BufferUsageFlags usage, vk::DeviceSize size);

// A host-visible buffer for data the GPU writes and the CPU reads, such as rendered
// frames. Prefers cached memory, which the CPU reads much faster.
Buffer create_readback_buffer(const VulkanContext& context, vk::DeviceSize size);

// Copies the start of a host-visible buffer into data. The GPU must be done writing it.
void read_host_buffer(const Buffer& buffer, std::span<std::byte> data);

// Copies data to the start of a host-visible buffer. The GPU must not be using it.
void write_host_buffer(const Buffer& buffer, std::span<const std::byte> data);

// The first of D32, X8_D24 and D16 that the device can use as a depth attachment.
vk::Format choose_depth_format(const vk::raii::PhysicalDevice& physical_device);

// A device-local image that is only rendered to within a frame, such as a depth
// buffer or a multisampled color buffer.
Image create_attachment_image(const VulkanContext& context, vk::Format format, vk::Extent2D extent,
                              vk::SampleCountFlagBits samples, vk::ImageUsageFlags usage,
                              vk::ImageAspectFlags aspect);

}  // namespace sim::render
