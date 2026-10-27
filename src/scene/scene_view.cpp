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
constexpr std::size_t obstacles_mesh = 1;
constexpr std::size_t first_vehicle_mesh = 2;

constexpr double ground_tile_size = 1.0;  // m
// Large enough that the edge is always past the fog in mesh.frag, which hides it.
constexpr int ground_tiles_per_side = 240;

// The follow camera looks at the vehicle from a fixed direction: south-west of it
// and above (the world frame is NED, so up is -z).
constexpr Vec3 follow_direction{-2.0, -0.9, -0.9};
constexpr double chase_elevation = 0.35;         // rad above the horizon, behind the vehicle.
constexpr double overhead_height_factor = 4.0;  // Times the camera distance.
constexpr double min_camera_height = 0.3;       // m
constexpr double vertical_fov = std::numbers::pi / 3.0;
constexpr double near_plane = 0.05;  // m
constexpr double far_plane = 200.0;  // m

constexpr std::array sky_color{0.42f, 0.60f, 0.85f, 1.0f};  // Also the fog color in mesh.frag.

}  // namespace

SceneView::SceneView(const CameraSettings& camera)
    : camera_{camera}, meshes_{make_ground_mesh(ground_tile_size, ground_tiles_per_side), render::Mesh{}} {}

bool SceneView::update_meshes(const runtime::Snapshot& snapshot) {
  // By identity, not version, since they may come from another simulator.
  if (snapshot.roster == roster_ && snapshot.obstacles == obstacles_) {
    return false;
  }
  if (snapshot.obstacles != obstacles_) {
    obstacles_ = snapshot.obstacles;
    meshes_[obstacles_mesh] = obstacles_ ? make_obstacles_mesh(*obstacles_) : render::Mesh{};
  }
  if (snapshot.roster != roster_) {
    roster_ = snapshot.roster;
    meshes_.resize(first_vehicle_mesh);
    for (const runtime::VehicleSpec& vehicle : roster_->vehicles) {
      meshes_.push_back(make_multirotor_mesh(vehicle.model));
    }
  }
  return true;
}

void SceneView::update_drawings(const ipc::VizScene& drawings) {
  if (have_drawings_ && drawings.version == drawings_version_) {
    return;
  }
  have_drawings_ = true;
  drawings_version_ = drawings.version;
  drawing_vertices_ = make_drawings_mesh(drawings).vertices;
}

void SceneView::forget_drawings() {
  have_drawings_ = false;
  drawing_vertices_.clear();
}

render::FrameView SceneView::frame(const runtime::Snapshot& snapshot, std::size_t follow,
                                   std::uint32_t width, std::uint32_t height) {
  const VehicleState* vehicle = follow < snapshot.vehicles.size() ? &snapshot.vehicles[follow].state : nullptr;
  const Vec3 target = vehicle != nullptr ? vehicle->position : Vec3{};
  const double distance = camera_.distance;

  Vec3 eye;
  Vec3 up{0.0, 0.0, -1.0};
  switch (camera_.mode) {
    case runtime::CameraMode::follow:
      eye = target + distance * normalized(follow_direction);
      break;
    case runtime::CameraMode::chase: {
      // Turns with the vehicle's heading, but not with its roll and pitch.
      const double yaw = vehicle != nullptr ? to_euler(vehicle->attitude).yaw : 0.0;
      eye = target + distance * Vec3{-std::cos(yaw) * std::cos(chase_elevation),
                                     -std::sin(yaw) * std::cos(chase_elevation), -std::sin(chase_elevation)};
      break;
    }
    case runtime::CameraMode::overhead:
      eye = target + Vec3{0.0, 0.0, -overhead_height_factor * distance};
      up = {1.0, 0.0, 0.0};  // North at the top of the image.
      break;
    case runtime::CameraMode::fixed:
      eye = camera_.position;
      break;
  }
  if (camera_.mode != runtime::CameraMode::fixed) {
    eye.z = std::min(eye.z, -min_camera_height);
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const Mat4 view_projection =
      multiply(perspective(vertical_fov, aspect, near_plane, far_plane), look_at(eye, target, up));

  // The ground is finite, so keep it centered under the followed vehicle. Moving it
  // in steps of two tiles keeps the checker pattern in place.
  const double ground_step = 2.0 * ground_tile_size;
  const Vec3 ground_offset{std::round(target.x / ground_step) * ground_step,
                           std::round(target.y / ground_step) * ground_step, 0.0};

  items_.clear();
  items_.push_back({.mesh = ground_mesh, .model = translation(ground_offset)});
  if (!meshes_[obstacles_mesh].vertices.empty()) {
    items_.push_back({.mesh = obstacles_mesh, .model = translation({})});
  }
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
