#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ipc/sim_server.hpp"
#include "platform/wayland/display.hpp"
#include "platform/wayland/window.hpp"
#include "render/renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/sim_runner.hpp"
#include "scene/scene_view.hpp"

namespace sim::viewer {

struct ViewerSettings {
  std::string follow;              // The vehicle followed at first; empty for the first one.
  std::uint32_t msaa_samples = 4;  // Rounded down to what the GPU supports; 1 disables MSAA.
  double camera_distance = 2.4;    // From the follow camera to the vehicle, m.
};

// Shows the latest simulation snapshot in a window, along with drawings received on
// the viz socket, and turns keyboard input into commands for the simulation. The
// camera follows one vehicle at a time, which is also the one the keys fly. It
// never touches the simulation directly, so it can't stall it: a slow or hidden
// window only means fewer frames are shown.
class Viewer {
 public:
  // Shows the drawings the server receives; it must outlive the viewer.
  Viewer(runtime::SimRunner& runner, const ipc::SimServer& server, const ViewerSettings& settings);

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;

  // Runs until the window is closed or Escape is pressed.
  void run();

 private:
  // Which flight keys are held down.
  struct HeldKeys {
    bool forward = false;
    bool back = false;
    bool left = false;
    bool right = false;
    bool up = false;
    bool down = false;
    bool yaw_left = false;
    bool yaw_right = false;
  };

  void handle_key(const platform::KeyEvent& event);
  // The index of the followed vehicle, after switching to another if it is gone.
  [[nodiscard]] std::size_t followed(const runtime::Roster& roster);
  void follow_next_vehicle(const runtime::Roster& roster);
  [[nodiscard]] runtime::PilotInput pilot_input() const;
  void update_title(const runtime::Snapshot& snapshot, std::size_t follow);

  runtime::SimRunner& runner_;
  std::string follow_;  // The vehicle's name, which outlasts others coming and going.
  scene::SceneView scene_;

  // Declaration order matters: Vulkan objects must be destroyed before the
  // Wayland surface and connection they were created from.
  platform::Display display_;
  platform::Window window_;
  render::VulkanContext vulkan_;
  render::Renderer renderer_;

  const ipc::SimServer& server_;

  HeldKeys keys_;
  runtime::PilotInput sent_input_;  // To the followed vehicle.
  bool follow_next_ = false;  // Tab was pressed.
  bool paused_ = false;
  bool quit_requested_ = false;
  std::chrono::steady_clock::time_point next_title_update_;
  double gpu_time_ms_ = 0.0;  // Averaged over the last title update interval.
};

}  // namespace sim::viewer
