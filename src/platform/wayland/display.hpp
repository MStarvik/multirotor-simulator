#pragma once

#include <cstdint>
#include <memory>

#include "platform/wayland/handle.hpp"
#include "platform/wayland/seat.hpp"

namespace sim::platform {

// The connection to the Wayland compositor and the globals it advertises.
// Required globals are validated at construction; optional ones may be null.
class Display {
 public:
  Display();
  ~Display();

  // Listeners hold a pointer to this object, so it must stay in place.
  Display(const Display&) = delete;
  Display& operator=(const Display&) = delete;

  // Reads and dispatches any events that are ready, without blocking.
  // Returns false if the connection to the compositor has been lost.
  [[nodiscard]] bool dispatch_pending();

  // Blocks until at least one event has been dispatched. Throws on connection loss.
  void dispatch_blocking();

  [[nodiscard]] wl_display* native() const { return display_.get(); }
  [[nodiscard]] wl_compositor* compositor() const { return compositor_.get(); }
  [[nodiscard]] xdg_wm_base* wm_base() const { return wm_base_.get(); }
  [[nodiscard]] zxdg_decoration_manager_v1* decoration_manager() const { return decoration_manager_.get(); }
  [[nodiscard]] wp_viewporter* viewporter() const { return viewporter_.get(); }
  [[nodiscard]] wp_fractional_scale_manager_v1* fractional_scale_manager() const {
    return fractional_scale_manager_.get();
  }
  [[nodiscard]] Seat* seat() const { return seat_.get(); }

 private:
  friend struct DisplayListeners;

  void handle_global(std::uint32_t name, const char* interface, std::uint32_t version);

  Handle<wl_display> display_;
  Handle<wl_registry> registry_;

  Handle<wl_compositor> compositor_;
  Handle<xdg_wm_base> wm_base_;
  Handle<zxdg_decoration_manager_v1> decoration_manager_;
  Handle<wp_viewporter> viewporter_;
  Handle<wp_fractional_scale_manager_v1> fractional_scale_manager_;
  Handle<wp_cursor_shape_manager_v1> cursor_shape_manager_;

  // Only the first seat is used. Its global is bound during registry enumeration
  // and wrapped in a Seat once all other globals are known.
  Handle<wl_seat> pending_seat_;
  std::unique_ptr<Seat> seat_;
};

}  // namespace sim::platform
