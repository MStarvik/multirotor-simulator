#include "platform/wayland/seat.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include <sys/mman.h>
#include <unistd.h>

#include <cursor-shape-v1-client-protocol.h>
#include <wayland-client.h>

#include "platform/wayland/listener.hpp"

namespace sim::platform {
namespace {

Seat& seat_from(void* data) { return *static_cast<Seat*>(data); }

}  // namespace

// Listener tables; a friend of Seat so the handlers can forward to its private members.
struct SeatListeners {
  static constexpr wl_seat_listener seat{
      .capabilities = [](void* data, wl_seat*, std::uint32_t capabilities) {
        seat_from(data).handle_capabilities(capabilities);
      },
      .name = ignore_event,
  };

  static constexpr wl_keyboard_listener keyboard{
      .keymap = [](void* data, wl_keyboard*, std::uint32_t format, int fd, std::uint32_t size) {
        seat_from(data).handle_keymap(format, fd, size);
      },
      .enter = ignore_event,
      .leave = [](void* data, wl_keyboard*, std::uint32_t /*serial*/, wl_surface*) {
        if (auto& handler = seat_from(data).on_keyboard_leave) {
          handler();
        }
      },
      .key = [](void* data, wl_keyboard*, std::uint32_t /*serial*/, std::uint32_t /*time*/,
                std::uint32_t key, std::uint32_t state) { seat_from(data).handle_key(key, state); },
      .modifiers = [](void* data, wl_keyboard*, std::uint32_t /*serial*/, std::uint32_t depressed,
                      std::uint32_t latched, std::uint32_t locked, std::uint32_t group) {
        seat_from(data).handle_modifiers(depressed, latched, locked, group);
      },
      .repeat_info = ignore_event,
  };

  static constexpr wl_pointer_listener pointer{
      .enter = [](void* data, wl_pointer*, std::uint32_t serial, wl_surface*, wl_fixed_t,
                  wl_fixed_t) { seat_from(data).handle_pointer_enter(serial); },
      .leave = ignore_event,
      .motion = ignore_event,
      .button = ignore_event,
      .axis = ignore_event,
      .frame = ignore_event,
      .axis_source = ignore_event,
      .axis_stop = ignore_event,
      .axis_discrete = ignore_event,
      .axis_value120 = ignore_event,
      .axis_relative_direction = ignore_event,
      .warp = ignore_event,
  };
};

Seat::Seat(Handle<wl_seat> seat, wp_cursor_shape_manager_v1* cursor_shape_manager)
    : cursor_shape_manager_{cursor_shape_manager},
      seat_{std::move(seat)},
      xkb_context_{xkb_context_new(XKB_CONTEXT_NO_FLAGS)} {
  if (!xkb_context_) {
    throw std::runtime_error{"failed to create xkbcommon context"};
  }
  wl_seat_add_listener(seat_.get(), &SeatListeners::seat, this);
}

Seat::~Seat() = default;

void Seat::handle_capabilities(std::uint32_t capabilities) {
  const bool has_keyboard = (capabilities & WL_SEAT_CAPABILITY_KEYBOARD) != 0;
  if (has_keyboard && !keyboard_) {
    keyboard_.reset(wl_seat_get_keyboard(seat_.get()));
    wl_keyboard_add_listener(keyboard_.get(), &SeatListeners::keyboard, this);
  } else if (!has_keyboard && keyboard_) {
    keyboard_.reset();
  }

  const bool has_pointer = (capabilities & WL_SEAT_CAPABILITY_POINTER) != 0;
  if (has_pointer && !pointer_) {
    pointer_.reset(wl_seat_get_pointer(seat_.get()));
    wl_pointer_add_listener(pointer_.get(), &SeatListeners::pointer, this);
    if (cursor_shape_manager_) {
      cursor_shape_device_.reset(
          wp_cursor_shape_manager_v1_get_pointer(cursor_shape_manager_, pointer_.get()));
    }
  } else if (!has_pointer && pointer_) {
    cursor_shape_device_.reset();
    pointer_.reset();
  }
}

void Seat::handle_keymap(std::uint32_t format, int fd, std::uint32_t size) {
  // We own the fd from here on. Since wl_seat v7 it must be mapped MAP_PRIVATE.
  void* data = format == WL_KEYBOARD_KEYMAP_FORMAT_XKB_V1
                   ? mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0)
                   : MAP_FAILED;
  close(fd);
  if (data == MAP_FAILED) {
    return;
  }

  const auto* text = static_cast<const char*>(data);
  Handle<xkb_keymap> keymap{xkb_keymap_new_from_buffer(xkb_context_.get(), text,
                                                       strnlen(text, size),
                                                       XKB_KEYMAP_FORMAT_TEXT_V1,
                                                       XKB_KEYMAP_COMPILE_NO_FLAGS)};
  munmap(data, size);
  if (!keymap) {
    return;
  }

  Handle<xkb_state> state{xkb_state_new(keymap.get())};
  if (!state) {
    return;
  }
  xkb_keymap_ = std::move(keymap);
  xkb_state_ = std::move(state);
}

void Seat::handle_key(std::uint32_t key, std::uint32_t state) {
  if (!xkb_state_ || !on_key) {
    return;
  }
  // Wayland sends evdev scancodes; XKB keycodes are offset by 8.
  const xkb_keysym_t keysym = xkb_state_key_get_one_sym(xkb_state_.get(), key + 8);
  on_key(KeyEvent{.keysym = keysym, .pressed = state == WL_KEYBOARD_KEY_STATE_PRESSED});
}

void Seat::handle_modifiers(std::uint32_t depressed, std::uint32_t latched, std::uint32_t locked,
                            std::uint32_t group) {
  if (xkb_state_) {
    xkb_state_update_mask(xkb_state_.get(), depressed, latched, locked, 0, 0, group);
  }
}

void Seat::handle_pointer_enter(std::uint32_t serial) {
  if (cursor_shape_device_) {
    wp_cursor_shape_device_v1_set_shape(cursor_shape_device_.get(), serial,
                                        WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_DEFAULT);
  }
}

}  // namespace sim::platform
