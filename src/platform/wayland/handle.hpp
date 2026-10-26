#pragma once

#include <memory>

// Owning smart pointers for Wayland and xkbcommon objects. The protocol headers
// stay out of this header: each type only needs a forward declaration and a
// destroy() overload, which is defined in handle.cpp.

struct wl_display;
struct wl_registry;
struct wl_compositor;
struct wl_surface;
struct wl_seat;
struct wl_keyboard;
struct wl_pointer;
struct xdg_wm_base;
struct xdg_surface;
struct xdg_toplevel;
struct zxdg_decoration_manager_v1;
struct zxdg_toplevel_decoration_v1;
struct wp_viewporter;
struct wp_viewport;
struct wp_fractional_scale_manager_v1;
struct wp_fractional_scale_v1;
struct wp_cursor_shape_manager_v1;
struct wp_cursor_shape_device_v1;
struct xkb_context;
struct xkb_keymap;
struct xkb_state;

namespace sim::platform {

void destroy(wl_display* object) noexcept;
void destroy(wl_registry* object) noexcept;
void destroy(wl_compositor* object) noexcept;
void destroy(wl_surface* object) noexcept;
void destroy(wl_seat* object) noexcept;
void destroy(wl_keyboard* object) noexcept;
void destroy(wl_pointer* object) noexcept;
void destroy(xdg_wm_base* object) noexcept;
void destroy(xdg_surface* object) noexcept;
void destroy(xdg_toplevel* object) noexcept;
void destroy(zxdg_decoration_manager_v1* object) noexcept;
void destroy(zxdg_toplevel_decoration_v1* object) noexcept;
void destroy(wp_viewporter* object) noexcept;
void destroy(wp_viewport* object) noexcept;
void destroy(wp_fractional_scale_manager_v1* object) noexcept;
void destroy(wp_fractional_scale_v1* object) noexcept;
void destroy(wp_cursor_shape_manager_v1* object) noexcept;
void destroy(wp_cursor_shape_device_v1* object) noexcept;
void destroy(xkb_context* object) noexcept;
void destroy(xkb_keymap* object) noexcept;
void destroy(xkb_state* object) noexcept;

struct Deleter {
  template <typename T>
  void operator()(T* object) const noexcept {
    destroy(object);
  }
};

template <typename T>
using Handle = std::unique_ptr<T, Deleter>;

}  // namespace sim::platform
