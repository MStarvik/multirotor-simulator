#pragma once

#include <cstdint>
#include <string>

#include "platform/wayland/handle.hpp"

namespace sim::platform {

class Display;

struct Extent {
  std::uint32_t width;
  std::uint32_t height;

  bool operator==(const Extent&) const = default;
};

// An xdg_toplevel window whose content is presented by Vulkan.
//
// The window tracks its logical size (from the compositor's configure events) and
// output scale, and derives the framebuffer extent the swapchain should use.
// Fractional scaling is used when the compositor supports it, with integer buffer
// scaling as the fallback.
class Window {
 public:
  // Creates the window and blocks until the compositor has sent its initial configure.
  Window(Display& display, const char* title, const char* app_id);
  ~Window();

  // Listeners hold a pointer to this object, so it must stay in place.
  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;

  [[nodiscard]] wl_surface* surface() const { return surface_.get(); }
  [[nodiscard]] bool close_requested() const { return close_requested_; }

  void set_title(const std::string& title);

  // Size of the swapchain images in pixels: the logical size times the output scale.
  [[nodiscard]] Extent framebuffer_extent() const;

  // Returns true, once, if the framebuffer extent has changed since the last call.
  [[nodiscard]] bool take_resize();

  // Sets the surface state that maps the framebuffer onto the logical window size.
  // This state is double-buffered and takes effect with the next wl_surface commit,
  // which vkQueuePresentKHR performs, so call it after recreating the swapchain and
  // before the first present at the new size.
  void apply_scale();

 private:
  friend struct WindowListeners;

  [[nodiscard]] bool uses_fractional_scale() const { return viewport_ != nullptr; }
  void handle_toplevel_configure(std::int32_t width, std::int32_t height);
  void handle_surface_configure(std::uint32_t serial);
  void handle_preferred_buffer_scale(std::int32_t scale);
  void handle_preferred_fractional_scale(std::uint32_t scale);

  // Declared parent-first so that children are destroyed before their parents.
  Handle<wl_surface> surface_;
  Handle<xdg_surface> xdg_surface_;
  Handle<xdg_toplevel> toplevel_;
  Handle<zxdg_toplevel_decoration_v1> decoration_;
  Handle<wp_viewport> viewport_;
  Handle<wp_fractional_scale_v1> fractional_scale_;

  Extent logical_size_{.width = 1280, .height = 720};
  Extent pending_logical_size_ = logical_size_;
  std::uint32_t scale_120_ = 120;  // Fractional scale in 1/120ths, as the protocol defines it.
  std::int32_t buffer_scale_ = 1;  // Integer scale, used without fractional scaling.

  bool configured_ = false;
  bool resized_ = false;
  bool close_requested_ = false;
};

}  // namespace sim::platform
