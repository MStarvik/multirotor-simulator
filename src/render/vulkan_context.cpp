#include "render/vulkan_context.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <optional>
#include <print>
#include <stdexcept>
#include <string_view>
#include <vector>

// Storage for Vulkan-Hpp's default dynamic dispatcher. The RAII classes carry their
// own dispatchers, but the default one must still be defined exactly once.
VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

namespace sim::render {
namespace {

#ifdef NDEBUG
constexpr bool want_validation = false;
#else
constexpr bool want_validation = true;
#endif

constexpr std::string_view validation_layer = "VK_LAYER_KHRONOS_validation";
constexpr std::uint32_t api_version = vk::ApiVersion14;

VKAPI_ATTR vk::Bool32 VKAPI_CALL debug_callback(vk::DebugUtilsMessageSeverityFlagBitsEXT severity,
                                                vk::DebugUtilsMessageTypeFlagsEXT /*types*/,
                                                const vk::DebugUtilsMessengerCallbackDataEXT* data,
                                                void* /*user_data*/) {
  std::println(stderr, "[vulkan {}] {}", vk::to_string(severity), data->pMessage);
  return vk::False;
}

bool has_layer(const vk::raii::Context& context, std::string_view name) {
  return std::ranges::any_of(context.enumerateInstanceLayerProperties(),
                             [&](const vk::LayerProperties& layer) {
                               return std::string_view{layer.layerName.data()} == name;
                             });
}

vk::raii::Instance create_instance(const vk::raii::Context& context, bool validation) {
  if (context.enumerateInstanceVersion() < api_version) {
    throw std::runtime_error{"the Vulkan loader does not support Vulkan 1.4"};
  }

  std::vector<const char*> layers;
  std::vector<const char*> extensions{vk::KHRSurfaceExtensionName, vk::KHRWaylandSurfaceExtensionName};
  if (validation) {
    layers.push_back(validation_layer.data());
    extensions.push_back(vk::EXTDebugUtilsExtensionName);
  }

  const vk::ApplicationInfo app_info{
      .pApplicationName = "Multirotor Simulator",
      .applicationVersion = vk::makeApiVersion(0, 0, 1, 0),
      .apiVersion = api_version,
  };
  return {context, vk::InstanceCreateInfo{
                       .pApplicationInfo = &app_info,
                       .enabledLayerCount = static_cast<std::uint32_t>(layers.size()),
                       .ppEnabledLayerNames = layers.data(),
                       .enabledExtensionCount = static_cast<std::uint32_t>(extensions.size()),
                       .ppEnabledExtensionNames = extensions.data(),
                   }};
}

vk::raii::DebugUtilsMessengerEXT create_debug_messenger(const vk::raii::Instance& instance) {
  using Severity = vk::DebugUtilsMessageSeverityFlagBitsEXT;
  using Type = vk::DebugUtilsMessageTypeFlagBitsEXT;
  return {instance, vk::DebugUtilsMessengerCreateInfoEXT{
                        .messageSeverity = Severity::eWarning | Severity::eError,
                        .messageType = Type::eGeneral | Type::eValidation | Type::ePerformance,
                        .pfnUserCallback = debug_callback,
                    }};
}

std::optional<std::uint32_t> find_queue_family(const vk::raii::PhysicalDevice& device,
                                               const vk::raii::SurfaceKHR& surface) {
  const auto families = device.getQueueFamilyProperties();
  for (std::uint32_t index = 0; index < families.size(); ++index) {
    if ((families[index].queueFlags & vk::QueueFlagBits::eGraphics) &&
        device.getSurfaceSupportKHR(index, *surface)) {
      return index;
    }
  }
  return std::nullopt;
}

bool supports_swapchain(const vk::raii::PhysicalDevice& device) {
  return std::ranges::any_of(device.enumerateDeviceExtensionProperties(),
                             [](const vk::ExtensionProperties& extension) {
                               return std::string_view{extension.extensionName.data()} ==
                                      vk::KHRSwapchainExtensionName;
                             });
}

}  // namespace

VulkanContext::VulkanContext(wl_display* display, wl_surface* surface) {
  const bool validation = want_validation && has_layer(context_, validation_layer);
  if (want_validation && !validation) {
    std::println(stderr, "warning: {} is not installed; running without validation",
                 validation_layer);
  }

  instance_ = create_instance(context_, validation);
  if (validation) {
    debug_messenger_ = create_debug_messenger(instance_);
  }
  surface_ = instance_.createWaylandSurfaceKHR({.display = display, .surface = surface});

  // Pick the first suitable device, preferring a discrete GPU.
  for (auto& candidate : instance_.enumeratePhysicalDevices()) {
    const auto properties = candidate.getProperties();
    const auto family = find_queue_family(candidate, surface_);
    if (properties.apiVersion < api_version || !family || !supports_swapchain(candidate)) {
      continue;
    }
    const bool discrete = properties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu;
    if (discrete || *physical_device_ == nullptr) {
      physical_device_ = std::move(candidate);
      queue_family_ = *family;
    }
    if (discrete) {
      break;
    }
  }
  if (*physical_device_ == nullptr) {
    throw std::runtime_error{"no Vulkan 1.4 device can present to this Wayland surface"};
  }
  std::println("Using {}", physical_device_.getProperties().deviceName.data());

  // These features are mandatory in Vulkan 1.3+, but must still be enabled.
  const vk::StructureChain<vk::PhysicalDeviceFeatures2, vk::PhysicalDeviceVulkan12Features,
                           vk::PhysicalDeviceVulkan13Features>
      features{
          {},
          {.timelineSemaphore = vk::True},
          {.synchronization2 = vk::True, .dynamicRendering = vk::True},
      };
  const float queue_priority = 1.0f;
  const vk::DeviceQueueCreateInfo queue_info{
      .queueFamilyIndex = queue_family_,
      .queueCount = 1,
      .pQueuePriorities = &queue_priority,
  };
  const std::array extensions{vk::KHRSwapchainExtensionName};

  device_ = physical_device_.createDevice({
      .pNext = &features.get<vk::PhysicalDeviceFeatures2>(),
      .queueCreateInfoCount = 1,
      .pQueueCreateInfos = &queue_info,
      .enabledExtensionCount = static_cast<std::uint32_t>(extensions.size()),
      .ppEnabledExtensionNames = extensions.data(),
  });
  queue_ = device_.getQueue(queue_family_, 0);
}

}  // namespace sim::render
