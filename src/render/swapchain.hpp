#pragma once

#include <cstdint>
#include <vector>

#include <vulkan/vulkan_raii.hpp>

namespace sim::render {

class VulkanContext;

// The swapchain and its per-image resources.
class Swapchain {
 public:
  Swapchain(const VulkanContext& context, vk::Extent2D extent);

  // Replaces the swapchain, handing the old one to the driver for a smooth transition.
  // The caller must make sure no submitted work still uses the old images.
  void recreate(vk::Extent2D extent);

  [[nodiscard]] const vk::raii::SwapchainKHR& handle() const { return swapchain_; }
  [[nodiscard]] vk::Format format() const { return format_; }
  [[nodiscard]] vk::Extent2D extent() const { return extent_; }
  [[nodiscard]] vk::Image image(std::uint32_t index) const { return images_[index]; }
  [[nodiscard]] vk::ImageView image_view(std::uint32_t index) const { return image_views_[index]; }
  [[nodiscard]] vk::Semaphore present_semaphore(std::uint32_t index) const {
    return present_semaphores_[index];
  }

 private:
  void create(vk::Extent2D extent);

  const VulkanContext& context_;
  vk::raii::SwapchainKHR swapchain_ = nullptr;
  vk::Format format_ = vk::Format::eUndefined;
  vk::Extent2D extent_;
  std::vector<vk::Image> images_;
  std::vector<vk::raii::ImageView> image_views_;
  // Signaled when rendering to an image completes and waited on by its present.
  // There is one per image rather than per frame in flight: a present's wait is only
  // known to have finished once the same image has been acquired again.
  std::vector<vk::raii::Semaphore> present_semaphores_;
};

}  // namespace sim::render
