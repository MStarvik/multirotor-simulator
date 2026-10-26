#pragma once

#include <cstdint>

#include <vulkan/vulkan_raii.hpp>

namespace sim::render {

// Owns the Vulkan instance, the Wayland presentation surface if any, and a logical
// device with a single graphics queue that can also present to that surface.
// Requires Vulkan 1.4 with dynamic rendering, synchronization2 and timeline semaphores.
class VulkanContext {
 public:
  // Without a display and surface, the context can only render offscreen, and works
  // without a compositor or a display attached.
  explicit VulkanContext(wl_display* display = nullptr, wl_surface* surface = nullptr);

  VulkanContext(const VulkanContext&) = delete;
  VulkanContext& operator=(const VulkanContext&) = delete;

  [[nodiscard]] const vk::raii::PhysicalDevice& physical_device() const { return physical_device_; }
  [[nodiscard]] const vk::raii::Device& device() const { return device_; }
  [[nodiscard]] const vk::raii::Queue& queue() const { return queue_; }
  [[nodiscard]] std::uint32_t queue_family() const { return queue_family_; }
  [[nodiscard]] const vk::raii::SurfaceKHR& surface() const { return surface_; }

 private:
  vk::raii::Context context_;
  vk::raii::Instance instance_ = nullptr;
  vk::raii::DebugUtilsMessengerEXT debug_messenger_ = nullptr;
  vk::raii::SurfaceKHR surface_ = nullptr;
  vk::raii::PhysicalDevice physical_device_ = nullptr;
  std::uint32_t queue_family_ = 0;
  vk::raii::Device device_ = nullptr;
  vk::raii::Queue queue_ = nullptr;
};

}  // namespace sim::render
