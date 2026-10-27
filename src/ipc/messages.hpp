#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <multirotor_sim/sim.h>

#include "runtime/sim_runner.hpp"

// Converts between simulator socket messages (multirotor_sim/sim.h) and the
// simulator's own types, for both ends of the socket.

namespace sim::ipc {

using Float3 = std::array<float, 3>;

struct VizMarker {
  Float3 position;
  Float3 color;
  float size;
};

struct VizPath {
  std::vector<Float3> points;
  Float3 color;
  float width;
};

using VizDrawing = std::variant<VizMarker, VizPath>;

// Everyone's drawings, by an id unique in the simulator.
struct VizScene {
  std::uint64_t version = 0;  // Increases with every change.
  std::map<std::uint32_t, VizDrawing> drawings;
};

// The largest message either end sends: a full path, or the most vehicles.
inline constexpr std::size_t max_message_size =
    std::max(sizeof(sim_viz_path) + SIM_VIZ_MAX_PATH_POINTS * sizeof(Float3),
             sizeof(sim_view_vehicles) + SIM_MAX_VEHICLES * sizeof(sim_view_vehicle));

template <typename T>
[[nodiscard]] T read_as(std::span<const std::byte> bytes) {
  T value;
  std::memcpy(&value, bytes.data(), sizeof value);  // memcpy, since the bytes may be unaligned.
  return value;
}

template <typename T>
[[nodiscard]] std::vector<std::byte> to_bytes(const T& message) {
  const auto bytes = std::as_bytes(std::span{&message, 1});
  return {bytes.begin(), bytes.end()};
}

// Up to the first NUL, or all of it.
[[nodiscard]] std::string read_name(const char (&name)[SIM_NAME_SIZE]);
// Truncates names that are too long.
void write_name(std::string_view name, char (&to)[SIM_NAME_SIZE]);

[[nodiscard]] sim_header make_header(std::uint32_t type, std::uint32_t id = 0);

// The rest throw std::invalid_argument for a malformed message.

// A SIM_VIZ_MARKER or SIM_VIZ_PATH message.
[[nodiscard]] VizDrawing decode_drawing(std::span<const std::byte> message);
[[nodiscard]] std::vector<std::byte> encode_drawing(std::uint32_t id, const VizDrawing& drawing);

// SIM_VIEW_VEHICLES. Only what a viewer needs survives the trip: names, controller
// kinds, and the rotors' layout and propeller size.
[[nodiscard]] std::vector<std::byte> encode_vehicles(const runtime::Roster& roster);
[[nodiscard]] std::shared_ptr<const runtime::Roster> decode_vehicles(std::span<const std::byte> message);

// SIM_VIEW_STATE. Only the time, pause, real-time factor and the vehicles'
// positions, attitudes and waiting survive the trip.
[[nodiscard]] std::vector<std::byte> encode_state(std::uint32_t request_id, const runtime::Snapshot& snapshot);
// Into a snapshot of the roster with the same version; returns false, leaving the
// snapshot alone, if the state is for another one.
[[nodiscard]] bool decode_state(std::span<const std::byte> message, const std::shared_ptr<const runtime::Roster>& roster,
                                runtime::Snapshot& snapshot);

}  // namespace sim::ipc
