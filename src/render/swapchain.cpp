#include "render/swapchain.hpp"

#include <algorithm>
#include <limits>
#include <span>

#include "render/vulkan_context.hpp"

namespace sim::render {
namespace {

vk::SurfaceFormatKHR choose_surface_format(std::span<const vk::SurfaceFormatKHR> formats) {
  for (const auto preferred : {vk::Format::eB8G8R8A8Srgb, vk::Format::eR8G8B8A8Srgb}) {
    const auto it = std::ranges::find_if(formats, [&](const vk::SurfaceFormatKHR& format) {
      return format.format == preferred && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear;
    });
    if (it != formats.end()) {
      return *it;
    }
  }
  return formats.front();
}

vk::CompositeAlphaFlagBitsKHR choose_composite_alpha(vk::CompositeAlphaFlagsKHR supported) {
  for (const auto mode : {vk::CompositeAlphaFlagBitsKHR::eOpaque,
                          vk::CompositeAlphaFlagBitsKHR::ePreMultiplied,
                          vk::CompositeAlphaFlagBitsKHR::ePostMultiplied}) {
    if (supported & mode) {
      return mode;
    }
  }
  return vk::CompositeAlphaFlagBitsKHR::eInherit;
}

}  // namespace

Swapchain::Swapchain(const VulkanContext& context, vk::Extent2D extent) : context_{context} {
  create(extent);
}

void Swapchain::recreate(vk::Extent2D extent) { create(extent); }

void Swapchain::create(vk::Extent2D extent) {
  const auto& physical_device = context_.physical_device();
  const auto& device = context_.device();
  const vk::SurfaceKHR surface = *context_.surface();

  const auto capabilities = physical_device.getSurfaceCapabilitiesKHR(surface);
  const auto surface_format = choose_surface_format(physical_device.getSurfaceFormatsKHR(surface));

  // On Wayland the surface has no fixed size (currentExtent is the special value
  // 0xFFFFFFFF), and the swapchain extent decides the buffer size.
  if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
    extent = capabilities.currentExtent;
  }
  extent.width = std::clamp(extent.width, capabilities.minImageExtent.width,
                            capabilities.maxImageExtent.width);
  extent.height = std::clamp(extent.height, capabilities.minImageExtent.height,
                             capabilities.maxImageExtent.height);

  std::uint32_t image_count = capabilities.minImageCount + 1;
  if (capabilities.maxImageCount != 0) {
    image_count = std::min(image_count, capabilities.maxImageCount);
  }

  // Constructing the new swapchain before the assignment destroys the old one,
  // which is still valid when passed as oldSwapchain.
  swapchain_ = device.createSwapchainKHR({
      .surface = surface,
      .minImageCount = image_count,
      .imageFormat = surface_format.format,
      .imageColorSpace = surface_format.colorSpace,
      .imageExtent = extent,
      .imageArrayLayers = 1,
      .imageUsage = vk::ImageUsageFlagBits::eColorAttachment,
      .imageSharingMode = vk::SharingMode::eExclusive,
      .preTransform = capabilities.currentTransform,
      .compositeAlpha = choose_composite_alpha(capabilities.supportedCompositeAlpha),
      .presentMode = vk::PresentModeKHR::eFifo,  // V-sync; the only mode guaranteed to exist.
      .clipped = vk::True,
      .oldSwapchain = *swapchain_,
  });
  format_ = surface_format.format;
  extent_ = extent;
  images_ = swapchain_.getImages();

  image_views_.clear();
  present_semaphores_.clear();
  for (const vk::Image image : images_) {
    image_views_.push_back(device.createImageView({
        .image = image,
        .viewType = vk::ImageViewType::e2D,
        .format = format_,
        .subresourceRange = {.aspectMask = vk::ImageAspectFlagBits::eColor,
                             .baseMipLevel = 0,
                             .levelCount = 1,
                             .baseArrayLayer = 0,
                             .layerCount = 1},
    }));
    present_semaphores_.push_back(device.createSemaphore({}));
  }
}

}  // namespace sim::render
