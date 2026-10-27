#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "ipc/sim_client.hpp"
#include "platform/wayland/display.hpp"
#include "platform/wayland/window.hpp"
#include "render/renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/sim_runner.hpp"
#include "scene/scene_view.hpp"

namespace sim::viewer {

struct ViewerSettings {
  std::filesystem::path socket;    // The simulator socket.
  std::string follow;              // The vehicle followed at first; empty for the first one.
  scene::CameraSettings camera;
  std::uint32_t msaa_samples = 4;  // Rounded down to what the GPU supports; 1 disables MSAA.
  bool exit_with_simulator = false;  // Rather than waiting for the next one.
};

// Shows a simulation in a window, along with everyone's drawings, and turns keyboard
// input into requests to the simulator. Talks to the simulator only over its
// socket, so any number of viewers can watch one simulation, each with its own
// camera, and none of them can stall it: a slow or hidden window only means fewer
// frames are shown. The camera follows one vehicle at a time, which is also the one
// the keys fly.
class Viewer {
 public:
  explicit Viewer(const ViewerSettings& settings);

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;

  // Runs until the window is closed or Escape is pressed, or with
  // exit_with_simulator, until the simulator exits.
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

  // Connects to the simulator if not connected, trying at most once a second.
  void connect();
  void handle_key(const platform::KeyEvent& event);
  // The index of the followed vehicle, after switching to another if it is gone.
  [[nodiscard]] std::size_t followed(const runtime::Roster& roster);
  void follow_next_vehicle(const runtime::Roster& roster);
  [[nodiscard]] runtime::PilotInput pilot_input() const;
  void update_title(const runtime::Snapshot& snapshot, std::size_t follow);

  ViewerSettings settings_;
  std::optional<ipc::SimClient> client_;
  std::chrono::steady_clock::time_point next_connect_attempt_;
  // Shown while not connected.
  const runtime::Snapshot no_simulation_{.roster = std::make_shared<const runtime::Roster>(),
                                         .obstacles = std::make_shared<const runtime::Obstacles>()};

  std::string follow_;  // The vehicle's name, which outlasts others coming and going.
  scene::SceneView scene_;

  // Declaration order matters: Vulkan objects must be destroyed before the
  // Wayland surface and connection they were created from.
  platform::Display display_;
  platform::Window window_;
  render::VulkanContext vulkan_;
  render::Renderer renderer_;

  HeldKeys keys_;
  runtime::PilotInput sent_input_;  // To the followed vehicle.
  bool follow_next_ = false;        // Tab was pressed.
  bool quit_requested_ = false;
  std::chrono::steady_clock::time_point next_title_update_;
  double gpu_time_ms_ = 0.0;  // Averaged over the last title update interval.
};

}  // namespace sim::viewer
