#include "ipc/messages.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace sim::ipc {
namespace {

static_assert(runtime::max_vehicles == SIM_MAX_VEHICLES);
static_assert(max_rotors == SIM_MAX_ROTORS);
static_assert(runtime::max_obstacles == SIM_MAX_OBSTACLES);

Float3 to_float3(const float (&v)[3]) { return {v[0], v[1], v[2]}; }

void copy(const Float3& from, float (&to)[3]) {
  to[0] = from[0];
  to[1] = from[1];
  to[2] = from[2];
}

void copy(const Vec3& from, double (&to)[3]) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

Vec3 to_vec3(const double (&v)[3]) { return {v[0], v[1], v[2]}; }

void expect(bool condition, const char* what) {
  if (!condition) {
    throw std::invalid_argument{what};
  }
}

// The array of `count` T after a header of type Header, checking the size adds up.
template <typename Header, typename T>
std::span<const std::byte> trailing(std::span<const std::byte> message, std::size_t count, const char* what) {
  expect(message.size() == sizeof(Header) + count * sizeof(T), what);
  return message.subspan(sizeof(Header));
}

}  // namespace

std::string read_name(const char (&name)[SIM_NAME_SIZE]) {
  return std::string{name, ::strnlen(name, SIM_NAME_SIZE)};
}

void write_name(std::string_view name, char (&to)[SIM_NAME_SIZE]) {
  std::memset(to, 0, sizeof to);
  std::memcpy(to, name.data(), std::min<std::size_t>(name.size(), SIM_NAME_SIZE));
}

sim_header make_header(std::uint32_t type, std::uint32_t id) {
  return {.type = type, .version = SIM_PROTOCOL_VERSION, .id = id, .reserved = 0};
}

VizDrawing decode_drawing(std::span<const std::byte> message) {
  expect(message.size() >= sizeof(sim_header), "a drawing too short for a header");
  switch (read_as<sim_header>(message).type) {
    case SIM_VIZ_MARKER: {
      expect(message.size() == sizeof(sim_viz_marker), "a marker of the wrong size");
      const auto marker = read_as<sim_viz_marker>(message);
      return VizMarker{
          .position = to_float3(marker.position),
          .color = to_float3(marker.color),
          .size = marker.size,
      };
    }
    case SIM_VIZ_PATH: {
      expect(message.size() >= sizeof(sim_viz_path), "a path of the wrong size");
      const auto path = read_as<sim_viz_path>(message);
      expect(path.point_count <= SIM_VIZ_MAX_PATH_POINTS, "a path with too many points");
      const auto points =
          trailing<sim_viz_path, Float3>(message, path.point_count, "a path whose size doesn't match its point count");
      VizPath drawing{.points = std::vector<Float3>(path.point_count),
                      .color = to_float3(path.color),
                      .width = path.width};
      std::memcpy(drawing.points.data(), points.data(), points.size());
      return drawing;
    }
    default:
      throw std::invalid_argument{"not a drawing"};
  }
}

std::vector<std::byte> encode_drawing(std::uint32_t id, const VizDrawing& drawing) {
  if (const auto* marker = std::get_if<VizMarker>(&drawing)) {
    sim_viz_marker message{};
    message.header = make_header(SIM_VIZ_MARKER, id);
    copy(marker->position, message.position);
    copy(marker->color, message.color);
    message.size = marker->size;
    return to_bytes(message);
  }
  const auto& path = std::get<VizPath>(drawing);
  sim_viz_path message{};
  message.header = make_header(SIM_VIZ_PATH, id);
  copy(path.color, message.color);
  message.width = path.width;
  message.point_count = static_cast<std::uint32_t>(path.points.size());
  std::vector<std::byte> bytes = to_bytes(message);
  const auto points = std::as_bytes(std::span{path.points});
  bytes.insert(bytes.end(), points.begin(), points.end());
  return bytes;
}

std::vector<std::byte> encode_vehicles(const runtime::Roster& roster) {
  const std::size_t count = std::min<std::size_t>(roster.vehicles.size(), SIM_MAX_VEHICLES);
  const sim_view_vehicles header{
      .header = make_header(SIM_VIEW_VEHICLES),
      .roster_version = roster.version,
      .count = static_cast<std::uint32_t>(count),
      .reserved = 0,
  };
  std::vector<std::byte> bytes = to_bytes(header);
  for (std::size_t i = 0; i < count; ++i) {
    const runtime::VehicleSpec& spec = roster.vehicles[i];
    sim_view_vehicle vehicle{};
    write_name(spec.name, vehicle.name);
    vehicle.rotor_count = static_cast<std::uint32_t>(std::min<std::size_t>(spec.model.rotors.size(), SIM_MAX_ROTORS));
    vehicle.propeller_radius = spec.model.propulsion.propeller_radius;
    for (std::size_t r = 0; r < vehicle.rotor_count; ++r) {
      copy(spec.model.rotors[r].position, vehicle.rotors[r].position);
      vehicle.rotors[r].spin = spec.model.rotors[r].spin;
    }
    const auto vehicle_bytes = to_bytes(vehicle);
    bytes.insert(bytes.end(), vehicle_bytes.begin(), vehicle_bytes.end());
  }
  return bytes;
}

std::shared_ptr<const runtime::Roster> decode_vehicles(std::span<const std::byte> message) {
  expect(message.size() >= sizeof(sim_view_vehicles), "a vehicle list too short for its header");
  const auto header = read_as<sim_view_vehicles>(message);
  expect(header.count <= SIM_MAX_VEHICLES, "a vehicle list with too many vehicles");
  const auto vehicles = trailing<sim_view_vehicles, sim_view_vehicle>(
      message, header.count, "a vehicle list whose size doesn't match its count");

  auto roster = std::make_shared<runtime::Roster>();
  roster->version = header.roster_version;
  for (std::size_t i = 0; i < header.count; ++i) {
    const auto vehicle = read_as<sim_view_vehicle>(vehicles.subspan(i * sizeof(sim_view_vehicle)));
    expect(vehicle.rotor_count <= SIM_MAX_ROTORS, "a vehicle with too many rotors");
    runtime::VehicleSpec spec{
        .name = read_name(vehicle.name),
        .model = {},
        .initial_state = {},
    };
    spec.model.propulsion.propeller_radius = vehicle.propeller_radius;
    for (std::size_t r = 0; r < vehicle.rotor_count; ++r) {
      spec.model.rotors.push_back({.position = to_vec3(vehicle.rotors[r].position), .spin = vehicle.rotors[r].spin});
    }
    roster->vehicles.push_back(std::move(spec));
  }
  return roster;
}

sim_obstacle encode_obstacle(const runtime::Obstacle& obstacle) {
  sim_obstacle wire{};
  write_name(obstacle.name, wire.name);
  wire.shape = obstacle.shape == runtime::ObstacleShape::cylinder ? SIM_OBSTACLE_CYLINDER : SIM_OBSTACLE_BOX;
  copy(obstacle.position, wire.position);
  copy(obstacle.size, wire.size);
  wire.yaw = obstacle.yaw;
  copy(obstacle.color, wire.color);
  return wire;
}

runtime::Obstacle decode_obstacle(const sim_obstacle& obstacle) {
  expect(obstacle.shape == SIM_OBSTACLE_BOX || obstacle.shape == SIM_OBSTACLE_CYLINDER, "an unknown obstacle shape");
  return {
      .name = read_name(obstacle.name),
      .shape = obstacle.shape == SIM_OBSTACLE_CYLINDER ? runtime::ObstacleShape::cylinder : runtime::ObstacleShape::box,
      .position = to_vec3(obstacle.position),
      .size = to_vec3(obstacle.size),
      .yaw = obstacle.yaw,
      .color = to_float3(obstacle.color),
  };
}

std::vector<std::byte> encode_obstacles(const runtime::Obstacles& obstacles) {
  const std::size_t count = std::min<std::size_t>(obstacles.obstacles.size(), SIM_MAX_OBSTACLES);
  const sim_view_obstacles header{
      .header = make_header(SIM_VIEW_OBSTACLES),
      .version = obstacles.version,
      .count = static_cast<std::uint32_t>(count),
      .reserved = 0,
  };
  std::vector<std::byte> bytes = to_bytes(header);
  for (std::size_t i = 0; i < count; ++i) {
    const auto obstacle_bytes = to_bytes(encode_obstacle(obstacles.obstacles[i]));
    bytes.insert(bytes.end(), obstacle_bytes.begin(), obstacle_bytes.end());
  }
  return bytes;
}

std::shared_ptr<const runtime::Obstacles> decode_obstacles(std::span<const std::byte> message) {
  expect(message.size() >= sizeof(sim_view_obstacles), "an obstacle list too short for its header");
  const auto header = read_as<sim_view_obstacles>(message);
  expect(header.count <= SIM_MAX_OBSTACLES, "an obstacle list with too many obstacles");
  const auto obstacles = trailing<sim_view_obstacles, sim_obstacle>(
      message, header.count, "an obstacle list whose size doesn't match its count");
  auto decoded = std::make_shared<runtime::Obstacles>();
  decoded->version = header.version;
  for (std::size_t i = 0; i < header.count; ++i) {
    decoded->obstacles.push_back(decode_obstacle(read_as<sim_obstacle>(obstacles.subspan(i * sizeof(sim_obstacle)))));
  }
  return decoded;
}

std::vector<std::byte> encode_state(std::uint32_t request_id, const runtime::Snapshot& snapshot) {
  const std::size_t count = std::min<std::size_t>(snapshot.vehicles.size(), SIM_MAX_VEHICLES);
  const sim_view_state header{
      .header = make_header(SIM_VIEW_STATE, request_id),
      .roster_version = snapshot.roster->version,
      .time = snapshot.time,
      .real_time_factor = snapshot.real_time_factor,
      .flags = snapshot.paused ? SIM_VIEW_PAUSED : 0u,
      .count = static_cast<std::uint32_t>(count),
  };
  std::vector<std::byte> bytes = to_bytes(header);
  for (std::size_t i = 0; i < count; ++i) {
    const runtime::VehicleSnapshot& vehicle = snapshot.vehicles[i];
    sim_view_vehicle_state state{};
    copy(vehicle.state.position, state.position);
    state.attitude[0] = vehicle.state.attitude.w;
    state.attitude[1] = vehicle.state.attitude.x;
    state.attitude[2] = vehicle.state.attitude.y;
    state.attitude[3] = vehicle.state.attitude.z;
    state.flags = vehicle.waiting_for_controller ? SIM_VIEW_WAITING : 0u;
    const auto state_bytes = to_bytes(state);
    bytes.insert(bytes.end(), state_bytes.begin(), state_bytes.end());
  }
  return bytes;
}

bool decode_state(std::span<const std::byte> message, const std::shared_ptr<const runtime::Roster>& roster,
                  runtime::Snapshot& snapshot) {
  expect(message.size() >= sizeof(sim_view_state), "a state too short for its header");
  const auto header = read_as<sim_view_state>(message);
  expect(header.count <= SIM_MAX_VEHICLES, "a state with too many vehicles");
  const auto states = trailing<sim_view_state, sim_view_vehicle_state>(
      message, header.count, "a state whose size doesn't match its count");
  if (roster == nullptr || header.roster_version != roster->version || header.count != roster->vehicles.size()) {
    return false;
  }

  snapshot.time = header.time;
  snapshot.real_time_factor = header.real_time_factor;
  snapshot.paused = (header.flags & SIM_VIEW_PAUSED) != 0;
  snapshot.roster = roster;
  snapshot.vehicles.resize(header.count);
  for (std::size_t i = 0; i < header.count; ++i) {
    const auto state = read_as<sim_view_vehicle_state>(states.subspan(i * sizeof(sim_view_vehicle_state)));
    runtime::VehicleSnapshot& vehicle = snapshot.vehicles[i];
    vehicle.state.position = to_vec3(state.position);
    vehicle.state.attitude = {state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]};
    vehicle.waiting_for_controller = (state.flags & SIM_VIEW_WAITING) != 0;
  }
  return true;
}

}  // namespace sim::ipc
