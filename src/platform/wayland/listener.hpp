#pragma once

// Private helpers for implementing Wayland listeners. Not part of the platform API.

namespace sim::platform {

// No-op handler for listener events we don't need. libwayland calls through every
// slot of a listener struct, so unused slots must still point at a function; a
// generic lambda converts to whatever function pointer type the slot has.
inline constexpr auto ignore_event = [](auto...) {};

}  // namespace sim::platform
