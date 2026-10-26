#include "platform/wayland/display.hpp"

#include <algorithm>
#include <cerrno>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <poll.h>

#include <cursor-shape-v1-client-protocol.h>
#include <fractional-scale-v1-client-protocol.h>
#include <viewporter-client-protocol.h>
#include <wayland-client.h>
#include <xdg-decoration-unstable-v1-client-protocol.h>
#include <xdg-shell-client-protocol.h>

#include "platform/wayland/listener.hpp"

namespace sim::platform {
namespace {

// Binds a global at the lower of the advertised version and the highest version
// whose events our listeners handle.
template <typename T>
Handle<T> bind(wl_registry* registry, std::uint32_t name, const wl_interface& interface,
               std::uint32_t advertised_version, std::uint32_t max_version) {
  const std::uint32_t version = std::min(advertised_version, max_version);
  return Handle<T>{static_cast<T*>(wl_registry_bind(registry, name, &interface, version))};
}

}  // namespace

struct DisplayListeners {
  static constexpr wl_registry_listener registry{
      .global = [](void* data, wl_registry*, std::uint32_t name, const char* interface,
                   std::uint32_t version) {
        static_cast<Display*>(data)->handle_global(name, interface, version);
      },
      .global_remove = ignore_event,
  };

  static constexpr xdg_wm_base_listener wm_base{
      .ping = [](void*, xdg_wm_base* wm_base, std::uint32_t serial) {
        xdg_wm_base_pong(wm_base, serial);
      },
  };
};

Display::Display() : display_{wl_display_connect(nullptr)} {
  if (!display_) {
    throw std::runtime_error{"failed to connect to the Wayland compositor"};
  }

  registry_.reset(wl_display_get_registry(display_.get()));
  wl_registry_add_listener(registry_.get(), &DisplayListeners::registry, this);
  if (wl_display_roundtrip(display_.get()) < 0) {
    throw std::runtime_error{"failed to enumerate Wayland globals"};
  }

  if (!compositor_) {
    throw std::runtime_error{"compositor does not support wl_compositor"};
  }
  if (!wm_base_) {
    throw std::runtime_error{"compositor does not support xdg_wm_base"};
  }

  if (pending_seat_) {
    seat_ = std::make_unique<Seat>(std::move(pending_seat_), cursor_shape_manager_.get());
  }
}

Display::~Display() = default;

void Display::handle_global(std::uint32_t name, const char* interface, std::uint32_t version) {
  wl_registry* registry = registry_.get();
  const std::string_view iface{interface};

  if (iface == wl_compositor_interface.name) {
    compositor_ = bind<wl_compositor>(registry, name, wl_compositor_interface, version, 6);
  } else if (iface == xdg_wm_base_interface.name) {
    wm_base_ = bind<xdg_wm_base>(registry, name, xdg_wm_base_interface, version, 6);
    xdg_wm_base_add_listener(wm_base_.get(), &DisplayListeners::wm_base, this);
  } else if (iface == zxdg_decoration_manager_v1_interface.name) {
    decoration_manager_ = bind<zxdg_decoration_manager_v1>(
        registry, name, zxdg_decoration_manager_v1_interface, version, 1);
  } else if (iface == wp_viewporter_interface.name) {
    viewporter_ = bind<wp_viewporter>(registry, name, wp_viewporter_interface, version, 1);
  } else if (iface == wp_fractional_scale_manager_v1_interface.name) {
    fractional_scale_manager_ = bind<wp_fractional_scale_manager_v1>(
        registry, name, wp_fractional_scale_manager_v1_interface, version, 1);
  } else if (iface == wp_cursor_shape_manager_v1_interface.name) {
    cursor_shape_manager_ = bind<wp_cursor_shape_manager_v1>(
        registry, name, wp_cursor_shape_manager_v1_interface, version, 2);
  } else if (iface == wl_seat_interface.name && !pending_seat_ && !seat_) {
    pending_seat_ = bind<wl_seat>(registry, name, wl_seat_interface, version, 9);
  }
}

bool Display::dispatch_pending() {
  wl_display* display = display_.get();

  // Standard non-blocking read: drain the queue, announce our intent to read,
  // flush outgoing requests, then read only if the socket has data.
  while (wl_display_prepare_read(display) != 0) {
    if (wl_display_dispatch_pending(display) < 0) {
      return false;
    }
  }

  if (wl_display_flush(display) < 0 && errno != EAGAIN) {
    wl_display_cancel_read(display);
    return false;
  }

  pollfd fd{.fd = wl_display_get_fd(display), .events = POLLIN, .revents = 0};
  if (poll(&fd, 1, 0) > 0) {
    if (wl_display_read_events(display) < 0) {
      return false;
    }
  } else {
    wl_display_cancel_read(display);
  }

  return wl_display_dispatch_pending(display) >= 0;
}

void Display::dispatch_blocking() {
  if (wl_display_dispatch(display_.get()) < 0) {
    throw std::runtime_error{"lost connection to the Wayland compositor"};
  }
}

}  // namespace sim::platform
