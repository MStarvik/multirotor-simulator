#include "render/memory.hpp"

#include <cstring>
#include <stdexcept>

#include "render/vulkan_context.hpp"

namespace sim::render {
namespace {

// The first memory type that has `required`, preferring one that also has `preferred`.
std::uint32_t find_memory_type(const vk::raii::PhysicalDevice& physical_device, std::uint32_t allowed_types,
                               vk::MemoryPropertyFlags required, vk::MemoryPropertyFlags preferred = {}) {
  const auto memory_properties = physical_device.getMemoryProperties();
  for (const vk::MemoryPropertyFlags wanted : {required | preferred, required}) {
    for (std::uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
      if ((allowed_types & (1u << i)) != 0 &&
          (memory_properties.memoryTypes[i].propertyFlags & wanted) == wanted) {
        return i;
      }
    }
  }
  throw std::runtime_error{"no suitable Vulkan memory type"};
}

vk::raii::DeviceMemory allocate(const VulkanContext& context, const vk::MemoryRequirements& requirements,
                                vk::MemoryPropertyFlags required, vk::MemoryPropertyFlags preferred = {}) {
  return context.device().allocateMemory({
      .allocationSize = requirements.size,
      .memoryTypeIndex = find_memory_type(context.physical_device(), requirements.memoryTypeBits, required,
                                          preferred),
  });
}

}  // namespace

Buffer create_host_buffer(const VulkanContext& context, vk::BufferUsageFlags usage, vk::DeviceSize size) {
  Buffer result;
  result.buffer = context.device().createBuffer({
      .size = size,
      .usage = usage,
      .sharingMode = vk::SharingMode::eExclusive,
  });
  result.memory = allocate(context, result.buffer.getMemoryRequirements(),
                           vk::MemoryPropertyFlagBits::eHostVisible |
                               vk::MemoryPropertyFlagBits::eHostCoherent);
  result.buffer.bindMemory(*result.memory, 0);
  return result;
}

Buffer create_readback_buffer(const VulkanContext& context, vk::DeviceSize size) {
  Buffer result;
  result.buffer = context.device().createBuffer({
      .size = size,
      .usage = vk::BufferUsageFlagBits::eTransferDst,
      .sharingMode = vk::SharingMode::eExclusive,
  });
  result.memory = allocate(context, result.buffer.getMemoryRequirements(),
                           vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                           vk::MemoryPropertyFlagBits::eHostCached);
  result.buffer.bindMemory(*result.memory, 0);
  return result;
}

void read_host_buffer(const Buffer& buffer, std::span<std::byte> data) {
  const void* mapped = buffer.memory.mapMemory(0, data.size());
  std::memcpy(data.data(), mapped, data.size());
  buffer.memory.unmapMemory();
}

void write_host_buffer(const Buffer& buffer, std::span<const std::byte> data) {
  void* mapped = buffer.memory.mapMemory(0, data.size());
  std::memcpy(mapped, data.data(), data.size());
  buffer.memory.unmapMemory();
}

vk::Format choose_depth_format(const vk::raii::PhysicalDevice& physical_device) {
  for (const auto format : {vk::Format::eD32Sfloat, vk::Format::eX8D24UnormPack32, vk::Format::eD16Unorm}) {
    if (physical_device.getFormatProperties(format).optimalTilingFeatures &
        vk::FormatFeatureFlagBits::eDepthStencilAttachment) {
      return format;
    }
  }
  throw std::runtime_error{"no supported depth format"};
}

Image create_attachment_image(const VulkanContext& context, vk::Format format, vk::Extent2D extent,
                              vk::SampleCountFlagBits samples, vk::ImageUsageFlags usage,
                              vk::ImageAspectFlags aspect) {
  const auto& device = context.device();
  Image result;
  result.image = device.createImage({
      .imageType = vk::ImageType::e2D,
      .format = format,
      .extent = {.width = extent.width, .height = extent.height, .depth = 1},
      .mipLevels = 1,
      .arrayLayers = 1,
      .samples = samples,
      .tiling = vk::ImageTiling::eOptimal,
      .usage = usage,
      .sharingMode = vk::SharingMode::eExclusive,
      .initialLayout = vk::ImageLayout::eUndefined,
  });
  result.memory = allocate(context, result.image.getMemoryRequirements(),
                           vk::MemoryPropertyFlagBits::eDeviceLocal);
  result.image.bindMemory(*result.memory, 0);
  result.view = device.createImageView({
      .image = *result.image,
      .viewType = vk::ImageViewType::e2D,
      .format = format,
      .subresourceRange = {.aspectMask = aspect,
                           .baseMipLevel = 0,
                           .levelCount = 1,
                           .baseArrayLayer = 0,
                           .layerCount = 1},
  });
  return result;
}

}  // namespace sim::render
