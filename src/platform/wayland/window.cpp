#include "platform/wayland/window.hpp"

#include <stdexcept>

#include <fractional-scale-v1-client-protocol.h>
#include <viewporter-client-protocol.h>
#include <wayland-client.h>
#include <xdg-decoration-unstable-v1-client-protocol.h>
#include <xdg-shell-client-protocol.h>

#include "platform/wayland/display.hpp"
#include "platform/wayland/listener.hpp"

namespace sim::platform {
namespace {

Window& window_from(void* data) { return *static_cast<Window*>(data); }

}  // namespace

struct WindowListeners {
  static constexpr wl_surface_listener surface{
      .enter = ignore_event,
      .leave = ignore_event,
      .preferred_buffer_scale = [](void* data, wl_surface*, std::int32_t scale) {
        window_from(data).handle_preferred_buffer_scale(scale);
      },
      .preferred_buffer_transform = ignore_event,
  };

  static constexpr xdg_surface_listener shell_surface{
      .configure = [](void* data, xdg_surface*, std::uint32_t serial) {
        window_from(data).handle_surface_configure(serial);
      },
  };

  static constexpr xdg_toplevel_listener toplevel{
      .configure = [](void* data, xdg_toplevel*, std::int32_t width, std::int32_t height,
                      wl_array* /*states*/) {
        window_from(data).handle_toplevel_configure(width, height);
      },
      .close = [](void* data, xdg_toplevel*) { window_from(data).close_requested_ = true; },
      .configure_bounds = ignore_event,
      .wm_capabilities = ignore_event,
  };

  static constexpr zxdg_toplevel_decoration_v1_listener decoration{
      .configure = ignore_event,
  };

  static constexpr wp_fractional_scale_v1_listener fractional_scale{
      .preferred_scale = [](void* data, wp_fractional_scale_v1*, std::uint32_t scale) {
        window_from(data).handle_preferred_fractional_scale(scale);
      },
  };
};

Window::Window(Display& display, const char* title, const char* app_id)
    : surface_{wl_compositor_create_surface(display.compositor())} {
  wl_surface_add_listener(surface_.get(), &WindowListeners::surface, this);

  xdg_surface_.reset(xdg_wm_base_get_xdg_surface(display.wm_base(), surface_.get()));
  xdg_surface_add_listener(xdg_surface_.get(), &WindowListeners::shell_surface, this);

  toplevel_.reset(xdg_surface_get_toplevel(xdg_surface_.get()));
  xdg_toplevel_add_listener(toplevel_.get(), &WindowListeners::toplevel, this);
  xdg_toplevel_set_title(toplevel_.get(), title);
  xdg_toplevel_set_app_id(toplevel_.get(), app_id);

  // Ask for server-side decorations. Compositors without xdg-decoration (e.g. GNOME)
  // expect client-side decorations, which would need libdecor.
  if (auto* manager = display.decoration_manager()) {
    decoration_.reset(zxdg_decoration_manager_v1_get_toplevel_decoration(manager, toplevel_.get()));
    zxdg_toplevel_decoration_v1_add_listener(decoration_.get(), &WindowListeners::decoration, this);
    zxdg_toplevel_decoration_v1_set_mode(decoration_.get(),
                                         ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE);
  }

  // Fractional scaling renders at the exact output resolution and lets the
  // viewport map the buffer back onto the logical size.
  if (display.fractional_scale_manager() && display.viewporter()) {
    viewport_.reset(wp_viewporter_get_viewport(display.viewporter(), surface_.get()));
    fractional_scale_.reset(wp_fractional_scale_manager_v1_get_fractional_scale(
        display.fractional_scale_manager(), surface_.get()));
    wp_fractional_scale_v1_add_listener(fractional_scale_.get(),
                                        &WindowListeners::fractional_scale, this);
  }

  // An initial commit without a buffer asks the compositor to configure the window.
  // No content may be attached (i.e. no Vulkan present) before that configure is acked.
  wl_surface_commit(surface_.get());
  while (!configured_) {
    display.dispatch_blocking();
  }
  resized_ = false;
}

Window::~Window() = default;

Extent Window::framebuffer_extent() const {
  if (uses_fractional_scale()) {
    // Rounded half away from zero, as fractional-scale-v1 specifies.
    return {.width = (logical_size_.width * scale_120_ + 60) / 120,
            .height = (logical_size_.height * scale_120_ + 60) / 120};
  }
  const auto scale = static_cast<std::uint32_t>(buffer_scale_);
  return {.width = logical_size_.width * scale, .height = logical_size_.height * scale};
}

void Window::set_title(const std::string& title) {
  xdg_toplevel_set_title(toplevel_.get(), title.c_str());
}

bool Window::take_resize() {
  const bool resized = resized_;
  resized_ = false;
  return resized;
}

void Window::apply_scale() {
  if (uses_fractional_scale()) {
    wp_viewport_set_destination(viewport_.get(), static_cast<std::int32_t>(logical_size_.width),
                                static_cast<std::int32_t>(logical_size_.height));
  } else {
    wl_surface_set_buffer_scale(surface_.get(), buffer_scale_);
  }
}

void Window::handle_toplevel_configure(std::int32_t width, std::int32_t height) {
  // Zero means the compositor leaves that dimension up to us; keep the current one.
  if (width > 0) {
    pending_logical_size_.width = static_cast<std::uint32_t>(width);
  }
  if (height > 0) {
    pending_logical_size_.height = static_cast<std::uint32_t>(height);
  }
}

void Window::handle_surface_configure(std::uint32_t serial) {
  // xdg_surface.configure ends a configure sequence; only now apply the pending state.
  xdg_surface_ack_configure(xdg_surface_.get(), serial);
  if (pending_logical_size_ != logical_size_) {
    logical_size_ = pending_logical_size_;
    resized_ = true;
  }
  configured_ = true;
}

void Window::handle_preferred_buffer_scale(std::int32_t scale) {
  if (!uses_fractional_scale() && scale > 0 && scale != buffer_scale_) {
    buffer_scale_ = scale;
    resized_ = true;
  }
}

void Window::handle_preferred_fractional_scale(std::uint32_t scale) {
  if (scale != scale_120_) {
    scale_120_ = scale;
    resized_ = true;
  }
}

}  // namespace sim::platform
