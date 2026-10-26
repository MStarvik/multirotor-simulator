#include "platform/wayland/handle.hpp"

#include <cursor-shape-v1-client-protocol.h>
#include <fractional-scale-v1-client-protocol.h>
#include <viewporter-client-protocol.h>
#include <wayland-client.h>
#include <xdg-decoration-unstable-v1-client-protocol.h>
#include <xdg-shell-client-protocol.h>
#include <xkbcommon/xkbcommon.h>

namespace sim::platform {

void destroy(wl_display* object) noexcept { wl_display_disconnect(object); }
void destroy(wl_registry* object) noexcept { wl_registry_destroy(object); }
void destroy(wl_compositor* object) noexcept { wl_compositor_destroy(object); }
void destroy(wl_surface* object) noexcept { wl_surface_destroy(object); }

// Input objects gained a release request in later versions, which also tells the
// compositor to stop sending events. Prefer it when the bound version has it.
void destroy(wl_seat* object) noexcept {
  if (wl_seat_get_version(object) >= WL_SEAT_RELEASE_SINCE_VERSION) {
    wl_seat_release(object);
  } else {
    wl_seat_destroy(object);
  }
}

void destroy(wl_keyboard* object) noexcept {
  if (wl_keyboard_get_version(object) >= WL_KEYBOARD_RELEASE_SINCE_VERSION) {
    wl_keyboard_release(object);
  } else {
    wl_keyboard_destroy(object);
  }
}

void destroy(wl_pointer* object) noexcept {
  if (wl_pointer_get_version(object) >= WL_POINTER_RELEASE_SINCE_VERSION) {
    wl_pointer_release(object);
  } else {
    wl_pointer_destroy(object);
  }
}

void destroy(xdg_wm_base* object) noexcept { xdg_wm_base_destroy(object); }
void destroy(xdg_surface* object) noexcept { xdg_surface_destroy(object); }
void destroy(xdg_toplevel* object) noexcept { xdg_toplevel_destroy(object); }
void destroy(zxdg_decoration_manager_v1* object) noexcept { zxdg_decoration_manager_v1_destroy(object); }
void destroy(zxdg_toplevel_decoration_v1* object) noexcept { zxdg_toplevel_decoration_v1_destroy(object); }
void destroy(wp_viewporter* object) noexcept { wp_viewporter_destroy(object); }
void destroy(wp_viewport* object) noexcept { wp_viewport_destroy(object); }
void destroy(wp_fractional_scale_manager_v1* object) noexcept { wp_fractional_scale_manager_v1_destroy(object); }
void destroy(wp_fractional_scale_v1* object) noexcept { wp_fractional_scale_v1_destroy(object); }
void destroy(wp_cursor_shape_manager_v1* object) noexcept { wp_cursor_shape_manager_v1_destroy(object); }
void destroy(wp_cursor_shape_device_v1* object) noexcept { wp_cursor_shape_device_v1_destroy(object); }
void destroy(xkb_context* object) noexcept { xkb_context_unref(object); }
void destroy(xkb_keymap* object) noexcept { xkb_keymap_unref(object); }
void destroy(xkb_state* object) noexcept { xkb_state_unref(object); }

}  // namespace sim::platform
