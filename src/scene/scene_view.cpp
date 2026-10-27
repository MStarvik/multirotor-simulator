#include "scene/scene_view.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "scene/meshes.hpp"
#include "scene/transform.hpp"

namespace sim::scene {
namespace {

// Indices into meshes_.
constexpr std::size_t ground_mesh = 0;
constexpr std::size_t first_vehicle_mesh = 1;

constexpr double ground_tile_size = 1.0;  // m
// Large enough that the edge is always past the fog in mesh.frag, which hides it.
constexpr int ground_tiles_per_side = 240;

// The camera looks at the vehicle from a fixed direction: south-west of it and
// above (the world frame is NED, so up is -z).
constexpr Vec3 camera_direction{-2.0, -0.9, -0.9};
constexpr double min_camera_height = 0.3;  // m
constexpr double vertical_fov = std::numbers::pi / 3.0;
constexpr double near_plane = 0.05;  // m
constexpr double far_plane = 200.0;  // m

constexpr std::array sky_color{0.42f, 0.60f, 0.85f, 1.0f};  // Also the fog color in mesh.frag.

}  // namespace

SceneView::SceneView(double camera_distance)
    : meshes_{make_ground_mesh(ground_tile_size, ground_tiles_per_side)},
      camera_offset_{camera_distance * normalized(camera_direction)} {}

bool SceneView::update_vehicles(const runtime::Roster& roster) {
  if (roster.version == roster_version_) {
    return false;
  }
  roster_version_ = roster.version;
  meshes_.resize(first_vehicle_mesh);
  for (const runtime::VehicleSpec& vehicle : roster.vehicles) {
    meshes_.push_back(make_multirotor_mesh(vehicle.model));
  }
  return true;
}

void SceneView::update_drawings(const ipc::SimServer& server) {
  if (server.drawings_version() == drawings_version_) {
    return;
  }
  const ipc::VizScene scene = server.drawings();
  drawings_version_ = scene.version;
  drawing_vertices_ = make_drawings_mesh(scene).vertices;
}

render::FrameView SceneView::frame(const runtime::Snapshot& snapshot, std::size_t follow,
                                   std::uint32_t width, std::uint32_t height) {
  const Vec3 target = follow < snapshot.vehicles.size() ? snapshot.vehicles[follow].state.position : Vec3{};
  const double aspect = static_cast<double>(width) / static_cast<double>(height);

  Vec3 eye = target + camera_offset_;
  eye.z = std::min(eye.z, -min_camera_height);
  const Mat4 view_projection = multiply(perspective(vertical_fov, aspect, near_plane, far_plane),
                                        look_at(eye, target, {0.0, 0.0, -1.0}));

  // The ground is finite, so keep it centered under the followed vehicle. Moving it
  // in steps of two tiles keeps the checker pattern in place.
  const double ground_step = 2.0 * ground_tile_size;
  const Vec3 ground_offset{std::round(target.x / ground_step) * ground_step,
                           std::round(target.y / ground_step) * ground_step, 0.0};

  items_.clear();
  items_.push_back({.mesh = ground_mesh, .model = translation(ground_offset)});
  // Only vehicles with meshes, in case update_vehicles() wasn't called.
  const std::size_t vehicles = std::min(snapshot.vehicles.size(), meshes_.size() - first_vehicle_mesh);
  for (std::size_t i = 0; i < vehicles; ++i) {
    const VehicleState& state = snapshot.vehicles[i].state;
    items_.push_back({.mesh = first_vehicle_mesh + i, .model = rigid_transform(state.position, state.attitude)});
  }
  return {
      .view_projection = view_projection,
      .clear_color = sky_color,
      .items = items_,
      .world_vertices = drawing_vertices_,
  };
}

}  // namespace sim::scene
