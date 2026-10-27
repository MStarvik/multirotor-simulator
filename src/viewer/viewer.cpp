#include "viewer/viewer.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>
#include <string>
#include <utility>

#include <xkbcommon/xkbcommon-keysyms.h>
#include <xkbcommon/xkbcommon.h>

namespace sim::viewer {
namespace {

constexpr auto title_interval = std::chrono::milliseconds{250};

vk::Extent2D to_vk(platform::Extent extent) {
  return {.width = extent.width, .height = extent.height};
}

}  // namespace

Viewer::Viewer(runtime::SimRunner& runner, const MultirotorModel& model, const ipc::SimServer& server,
               const ViewerSettings& settings)
    : runner_{runner},
      follow_{settings.follow},
      scene_{model, settings.camera_distance},
      window_{display_, "Multirotor Simulator", "multirotor-sim"},
      vulkan_{display_.native(), window_.surface()},
      renderer_{vulkan_, to_vk(window_.framebuffer_extent()), settings.msaa_samples, scene_.meshes()},
      server_{server} {
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

    const runtime::Snapshot& snapshot = runner_.latest();
    if (std::exchange(follow_next_, false)) {
      follow_next_vehicle(*snapshot.roster);
    }
    const std::size_t follow = followed(*snapshot.roster);
    if (const runtime::PilotInput input = pilot_input(); input != sent_input_ && !follow_.empty()) {
      runner_.push(runtime::SetPilotInput{.vehicle = follow_, .input = input});
      sent_input_ = input;
    }

    scene_.update_drawings(server_);
    paused_ = snapshot.paused;
    update_title(snapshot, follow);
    // Presentation is FIFO, so this blocks at the display's refresh rate. That only
    // slows down this loop; the simulation runs on its own thread.
    const vk::Extent2D extent = renderer_.extent();
    renderer_.draw_frame(scene_.frame(snapshot, follow, extent.width, extent.height));
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
      case XKB_KEY_Tab:
        follow_next_ = true;  // Done with the next snapshot, which has the current vehicles.
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
  // Gone, or not chosen yet: follow the first, if there is any.
  follow_ = vehicles.empty() ? std::string{} : vehicles.front().name;
  sent_input_ = {};
  return 0;
}

void Viewer::follow_next_vehicle(const runtime::Roster& roster) {
  if (roster.vehicles.empty()) {
    return;
  }
  // Let go of the sticks of the vehicle left behind. The held keys are sent to the
  // new one right after.
  if (sent_input_ != runtime::PilotInput{} && !follow_.empty()) {
    runner_.push(runtime::SetPilotInput{.vehicle = follow_, .input = {}});
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
  window_.set_title(std::format(
      "Multirotor Simulator — {}{:.1f} s, {:.2f}× real time{} | {}×{}, {}× MSAA, GPU {:.3f} ms", vehicle,
      snapshot.time, snapshot.real_time_factor, status, extent.width, extent.height, renderer_.msaa_samples(),
      gpu_time_ms_));
}

}  // namespace sim::viewer
