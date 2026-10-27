#include "viewer/viewer.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <xkbcommon/xkbcommon-keysyms.h>
#include <xkbcommon/xkbcommon.h>

namespace sim::viewer {
namespace {

constexpr auto title_interval = std::chrono::milliseconds{250};
constexpr auto connect_interval = std::chrono::seconds{1};

runtime::CameraMode next_camera_mode(runtime::CameraMode mode) {
  switch (mode) {
    case runtime::CameraMode::follow: return runtime::CameraMode::chase;
    case runtime::CameraMode::chase: return runtime::CameraMode::overhead;
    case runtime::CameraMode::overhead: return runtime::CameraMode::fixed;
    case runtime::CameraMode::fixed: return runtime::CameraMode::follow;
  }
  return runtime::CameraMode::follow;
}

vk::Extent2D to_vk(platform::Extent extent) {
  return {.width = extent.width, .height = extent.height};
}

}  // namespace

Viewer::Viewer(const ViewerSettings& settings)
    : settings_{settings},
      follow_{settings.follow},
      scene_{settings.camera},
      window_{display_, "Multirotor Simulator", "multirotor-sim"},
      vulkan_{display_.native(), window_.surface()},
      renderer_{vulkan_, to_vk(window_.framebuffer_extent()), settings.msaa_samples, scene_.meshes()} {
  window_.apply_scale();
  connect();
  if (!client_ && settings_.exit_with_simulator) {
    throw std::runtime_error{std::format("no simulator at {}", settings_.socket.native())};
  }

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

    // The state asked for last frame, which arrived meanwhile. A frame behind, but
    // waiting for it would hold up the frame.
    if (client_ && !client_->receive()) {
      client_.reset();
      if (settings_.exit_with_simulator) {
        return;
      }
      scene_.forget_drawings();
    }
    connect();
    if (client_) {
      client_->request_state();
    }

    const runtime::Snapshot& snapshot = client_ ? client_->snapshot() : no_simulation_;
    if (std::exchange(follow_next_, false)) {
      follow_next_vehicle(*snapshot.roster);
    }
    const std::size_t follow = followed(*snapshot.roster);
    if (const runtime::PilotInput input = pilot_input(); input != sent_input_ && client_ && !follow_.empty()) {
      client_->set_pilot_input(follow_, input);
      sent_input_ = input;
    }

    if (scene_.update_meshes(snapshot)) {
      renderer_.set_meshes(scene_.meshes());
    }
    if (client_) {
      scene_.update_drawings(client_->drawings());
    }
    update_title(snapshot, follow);
    // Presentation is FIFO, so this blocks at the display's refresh rate, which
    // only slows down this loop: the simulation runs in another process.
    const vk::Extent2D extent = renderer_.extent();
    renderer_.draw_frame(scene_.frame(snapshot, follow, extent.width, extent.height));
  }
}

void Viewer::connect() {
  const auto now = std::chrono::steady_clock::now();
  if (client_ || now < next_connect_attempt_) {
    return;
  }
  next_connect_attempt_ = now + connect_interval;
  try {
    client_.emplace(settings_.socket);
    sent_input_ = {};  // The new simulator has none.
  } catch (const std::system_error&) {
    // Not running (yet); try again later.
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
        if (client_) {
          client_->set_paused(!client_->snapshot().paused);
        }
        return;
      case XKB_KEY_r:
        if (client_) {
          client_->reset();
          sent_input_ = {};  // The controller forgets its input on reset.
        }
        return;
      case XKB_KEY_Tab:
        follow_next_ = true;  // Done with the next snapshot, which has the current vehicles.
        return;
      case XKB_KEY_c:
        scene_.set_camera_mode(next_camera_mode(scene_.camera_mode()));
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

std::size_t Viewer::followed(const runtime::Roster& roster) {
  const auto& vehicles = roster.vehicles;
  const auto found = std::ranges::find(vehicles, follow_, &runtime::VehicleSpec::name);
  if (found != vehicles.end()) {
    return static_cast<std::size_t>(found - vehicles.begin());
  }
  // Gone, or not chosen yet: follow the first. Without vehicles, such as before
  // they arrive, keep the name in case it comes.
  if (!vehicles.empty()) {
    follow_ = vehicles.front().name;
    sent_input_ = {};
  }
  return 0;
}

void Viewer::follow_next_vehicle(const runtime::Roster& roster) {
  if (roster.vehicles.empty()) {
    return;
  }
  // Let go of the sticks of the vehicle left behind. The held keys are sent to the
  // new one right after.
  if (sent_input_ != runtime::PilotInput{} && client_) {
    client_->set_pilot_input(follow_, {});
  }
  sent_input_ = {};
  follow_ = roster.vehicles[(followed(roster) + 1) % roster.vehicles.size()].name;
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

void Viewer::update_title(const runtime::Snapshot& snapshot, std::size_t follow) {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_title_update_) {
    return;
  }
  next_title_update_ = now + title_interval;
  if (const auto gpu_time = renderer_.take_average_gpu_time_ms()) {
    gpu_time_ms_ = *gpu_time;
  }
  std::string status;
  if (snapshot.paused) {
    status = " (paused)";
  } else {
    std::string waiting;
    for (std::size_t i = 0; i < snapshot.vehicles.size(); ++i) {
      if (snapshot.vehicles[i].waiting_for_controller) {
        waiting += std::format("{}{}", waiting.empty() ? "" : ", ", snapshot.roster->vehicles[i].name);
      }
    }
    if (!waiting.empty()) {
      status = std::format(" (waiting for the flight controller of {})", waiting);
    }
  }
  // Only name the vehicle when there is a choice.
  std::string vehicle;
  if (follow < snapshot.vehicles.size()) {
    if (snapshot.vehicles.size() > 1) {
      vehicle = std::format("vehicle {}, ", follow_);
    }
    // 0.0 - z avoids printing -0.00.
    vehicle += std::format("altitude {:.2f} m, ", 0.0 - snapshot.vehicles[follow].state.position.z);
  }
  const vk::Extent2D extent = renderer_.extent();
  const std::string details = std::format("{} camera | {}×{}, {}× MSAA, GPU {:.3f} ms", to_string(scene_.camera_mode()),
                                          extent.width, extent.height, renderer_.msaa_samples(), gpu_time_ms_);
  if (!client_) {
    window_.set_title(std::format("Multirotor Simulator — waiting for a simulator at {} | {}",
                                  settings_.socket.native(), details));
    return;
  }
  window_.set_title(std::format("Multirotor Simulator — {}{:.1f} s, {:.2f}× real time{} | {}", vehicle,
                                snapshot.time, snapshot.real_time_factor, status, details));
}

}  // namespace sim::viewer
