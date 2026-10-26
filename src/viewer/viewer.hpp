#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <vector>

#include "ipc/viz_server.hpp"
#include "platform/wayland/display.hpp"
#include "platform/wayland/window.hpp"
#include "render/renderer.hpp"
#include "render/vulkan_context.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

namespace sim::viewer {

// Shows the latest simulation snapshot in a window, along with drawings received on
// the viz socket, and turns keyboard input into commands for the simulation. It
// never touches the simulation directly, so it can't stall it: a slow or hidden
// window only means fewer frames are shown.
class Viewer {
 public:
  Viewer(runtime::SimRunner& runner, const MultirotorModel& model, std::uint32_t msaa_samples,
         std::filesystem::path viz_socket);

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
  [[nodiscard]] runtime::PilotInput pilot_input() const;
  void update_drawings();
  void update_title(const runtime::Snapshot& snapshot);
  void draw(const runtime::Snapshot& snapshot);

  runtime::SimRunner& runner_;

  // Declaration order matters: Vulkan objects must be destroyed before the
  // Wayland surface and connection they were created from.
  platform::Display display_;
  platform::Window window_;
  render::VulkanContext vulkan_;
  render::Renderer renderer_;

  ipc::VizServer viz_server_;
  std::uint64_t drawings_version_ = 0;
  std::vector<render::Vertex> drawing_vertices_;

  HeldKeys keys_;
  runtime::PilotInput sent_input_;
  bool paused_ = false;
  bool quit_requested_ = false;
  std::chrono::steady_clock::time_point next_title_update_;
  double gpu_time_ms_ = 0.0;  // Averaged over the last title update interval.
};

}  // namespace sim::viewer
