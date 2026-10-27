#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ipc/viz_server.hpp"
#include "platform/wayland/display.hpp"
#include "platform/wayland/window.hpp"
#include "render/renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/sim_runner.hpp"
#include "scene/scene_view.hpp"
#include "sim/multirotor.hpp"

namespace sim::viewer {

struct ViewerSettings {
  std::vector<std::string> vehicle_names;  // In the simulation's order.
  std::size_t follow = 0;          // The vehicle followed at first.
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
  Viewer(runtime::SimRunner& runner, const MultirotorModel& model, const ipc::VizServer& viz_server,
         const ViewerSettings& settings);

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
  void follow_next_vehicle();
  [[nodiscard]] runtime::PilotInput pilot_input() const;
  void update_title(const runtime::Snapshot& snapshot);

  runtime::SimRunner& runner_;
  std::vector<std::string> vehicle_names_;
  std::size_t follow_;
  scene::SceneView scene_;

  // Declaration order matters: Vulkan objects must be destroyed before the
  // Wayland surface and connection they were created from.
  platform::Display display_;
  platform::Window window_;
  render::VulkanContext vulkan_;
  render::Renderer renderer_;

  const ipc::VizServer& viz_server_;

  HeldKeys keys_;
  runtime::PilotInput sent_input_;  // To the followed vehicle.
  bool paused_ = false;
  bool quit_requested_ = false;
  std::chrono::steady_clock::time_point next_title_update_;
  double gpu_time_ms_ = 0.0;  // Averaged over the last title update interval.
};

}  // namespace sim::viewer
