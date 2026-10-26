#pragma once

#include <cstdint>
#include <functional>

#include <xkbcommon/xkbcommon.h>

#include "platform/wayland/handle.hpp"

namespace sim::platform {

struct KeyEvent {
  xkb_keysym_t keysym;
  bool pressed;
};

// A wl_seat with its keyboard and pointer. Keyboard input is translated to keysyms
// with xkbcommon; the pointer only gets a default cursor for now.
class Seat {
 public:
  Seat(Handle<wl_seat> seat, wp_cursor_shape_manager_v1* cursor_shape_manager);
  ~Seat();

  Seat(const Seat&) = delete;
  Seat& operator=(const Seat&) = delete;

  std::function<void(const KeyEvent&)> on_key;
  // Keyboard focus was lost; no release events will arrive for keys still held.
  std::function<void()> on_keyboard_leave;

 private:
  friend struct SeatListeners;

  void handle_capabilities(std::uint32_t capabilities);
  void handle_keymap(std::uint32_t format, int fd, std::uint32_t size);
  void handle_key(std::uint32_t key, std::uint32_t state);
  void handle_modifiers(std::uint32_t depressed, std::uint32_t latched, std::uint32_t locked,
                        std::uint32_t group);
  void handle_pointer_enter(std::uint32_t serial);

  wp_cursor_shape_manager_v1* cursor_shape_manager_;

  // Declared parent-first so that children are destroyed before their parents.
  Handle<wl_seat> seat_;
  Handle<wl_keyboard> keyboard_;
  Handle<wl_pointer> pointer_;
  Handle<wp_cursor_shape_device_v1> cursor_shape_device_;

  Handle<xkb_context> xkb_context_;
  Handle<xkb_keymap> xkb_keymap_;
  Handle<xkb_state> xkb_state_;
};

}  // namespace sim::platform
