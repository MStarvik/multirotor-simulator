#include "viewer/viewer.hpp"

#include <array>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <xkbcommon/xkbcommon-keysyms.h>
#include <xkbcommon/xkbcommon.h>

#include "viewer/meshes.hpp"
#include "viewer/transform.hpp"

namespace sim::viewer {
namespace {

// Indices into the meshes the renderer is created with.
constexpr std::size_t ground_mesh = 0;
constexpr std::size_t vehicle_mesh = 1;

constexpr double ground_tile_size = 1.0;  // m
constexpr int ground_tiles_per_side = 80;

// The follow camera sits at a fixed offset from the vehicle: south-west of it and
// above (the world frame is NED, so up is -z).
constexpr Vec3 camera_offset{-2.0, -0.9, -0.9};
constexpr double min_camera_height = 0.3;  // m

constexpr std::array sky_color{0.42f, 0.60f, 0.85f, 1.0f};
constexpr auto title_interval = std::chrono::milliseconds{250};

std::array<render::Mesh, 2> make_scene_meshes(const MultirotorModel& model) {
  return {make_ground_mesh(ground_tile_size, ground_tiles_per_side), make_multirotor_mesh(model)};
}

vk::Extent2D to_vk(platform::Extent extent) {
  return {.width = extent.width, .height = extent.height};
}

}  // namespace

Viewer::Viewer(runtime::SimRunner& runner, const MultirotorModel& model, std::uint32_t msaa_samples,
               std::filesystem::path viz_socket)
    : runner_{runner},
      window_{display_, "Multirotor Simulator", "multirotor-sim"},
      vulkan_{display_.native(), window_.surface()},
      renderer_{vulkan_, to_vk(window_.framebuffer_extent()), msaa_samples, make_scene_meshes(model)},
      viz_server_{std::move(viz_socket)} {
  window_.apply_scale();

  if (auto* seat = display_.seat()) {
    seat->on_key = [this](const platform::KeyEvent& event) { handle_key(event); };
    seat->on_keyboard_leave = [this] { keys_ = {}; };
  }
}

void Viewer::run() {
  while (!quit_requested_ && !window_.close_requested()) {
    if (!display_.dispatch_pending()) {
      throw std::runtime_error{"lost connection to the Wayland compositor"};
    }

    if (window_.take_resize()) {
      renderer_.resize(to_vk(window_.framebuffer_extent()));
      window_.apply_scale();
    }

    if (const runtime::PilotInput input = pilot_input(); input != sent_input_) {
      runner_.push(runtime::SetPilotInput{input});
      sent_input_ = input;
    }

    update_drawings();
    const runtime::Snapshot& snapshot = runner_.latest();
    paused_ = snapshot.paused;
    update_title(snapshot);
    // Presentation is FIFO, so this blocks at the display's refresh rate. That only
    // slows down this loop; the simulation runs on its own thread.
    draw(snapshot);
  }
}

void Viewer::handle_key(const platform::KeyEvent& event) {
  const xkb_keysym_t keysym = xkb_keysym_to_lower(event.keysym);

  if (event.pressed) {
    switch (keysym) {
      case XKB_KEY_Escape:
        quit_requested_ = true;
        return;
      case XKB_KEY_space:
        runner_.push(runtime::SetPaused{.paused = !paused_});
        return;
      case XKB_KEY_r:
        runner_.push(runtime::Reset{});
        sent_input_ = {};  // The controller forgets its input on reset.
        return;
      default:
        break;
    }
  }

  switch (keysym) {
    case XKB_KEY_Up: keys_.forward = event.pressed; break;
    case XKB_KEY_Down: keys_.back = event.pressed; break;
    case XKB_KEY_Left: keys_.left = event.pressed; break;
    case XKB_KEY_Right: keys_.right = event.pressed; break;
    case XKB_KEY_w: keys_.up = event.pressed; break;
    case XKB_KEY_s: keys_.down = event.pressed; break;
    case XKB_KEY_a: keys_.yaw_left = event.pressed; break;
    case XKB_KEY_d: keys_.yaw_right = event.pressed; break;
    default: break;
  }
}

runtime::PilotInput Viewer::pilot_input() const {
  const auto axis = [](bool positive, bool negative) { return (positive ? 1.0 : 0.0) - (negative ? 1.0 : 0.0); };
  return {
      .roll = axis(keys_.right, keys_.left),
      .pitch = axis(keys_.forward, keys_.back),
      .yaw = axis(keys_.yaw_right, keys_.yaw_left),
      .throttle = axis(keys_.up, keys_.down),
  };
}

void Viewer::update_drawings() {
  if (viz_server_.version() == drawings_version_) {
    return;
  }
  const ipc::VizScene scene = viz_server_.scene();
  drawings_version_ = scene.version;
  drawing_vertices_ = make_drawings_mesh(scene).vertices;
}

void Viewer::update_title(const runtime::Snapshot& snapshot) {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_title_update_) {
    return;
  }
  next_title_update_ = now + title_interval;
  if (const auto gpu_time = renderer_.take_average_gpu_time_ms()) {
    gpu_time_ms_ = *gpu_time;
  }
  const char* status = snapshot.paused                   ? " (paused)"
                       : snapshot.waiting_for_controller ? " (waiting for flight controller)"
                                                         : "";
  const vk::Extent2D extent = renderer_.extent();
  window_.set_title(std::format(
      "Multirotor Simulator — {:.1f} s, altitude {:.2f} m, {:.2f}× real time{} | {}×{}, {}× MSAA, GPU {:.3f} ms",
      snapshot.time, 0.0 - snapshot.vehicle.position.z,  // Avoids printing -0.00.
      snapshot.real_time_factor, status, extent.width, extent.height, renderer_.msaa_samples(), gpu_time_ms_));
}

void Viewer::draw(const runtime::Snapshot& snapshot) {
  const VehicleState& vehicle = snapshot.vehicle;
  const vk::Extent2D extent = renderer_.extent();
  const double aspect = static_cast<double>(extent.width) / static_cast<double>(extent.height);

  Vec3 eye = vehicle.position + camera_offset;
  eye.z = std::min(eye.z, -min_camera_height);
  const Mat4 view_projection =
      multiply(perspective(std::numbers::pi / 3.0, aspect, 0.05, 200.0),
               look_at(eye, vehicle.position, {0.0, 0.0, -1.0}));

  // The ground is finite, so keep it centered under the vehicle. Moving it in steps
  // of two tiles keeps the checker pattern in place.
  const double ground_step = 2.0 * ground_tile_size;
  const Vec3 ground_offset{std::round(vehicle.position.x / ground_step) * ground_step,
                           std::round(vehicle.position.y / ground_step) * ground_step, 0.0};

  const std::array items{
      render::DrawItem{.mesh = ground_mesh, .model = translation(ground_offset)},
      render::DrawItem{.mesh = vehicle_mesh, .model = rigid_transform(vehicle.position, vehicle.attitude)},
  };
  renderer_.draw_frame({
      .view_projection = view_projection,
      .clear_color = sky_color,
      .items = items,
      .world_vertices = drawing_vertices_,
  });
}

}  // namespace sim::viewer
