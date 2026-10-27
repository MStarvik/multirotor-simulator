#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "ipc/messages.hpp"
#include "render/scene.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"

namespace sim::scene {

struct CameraSettings {
  runtime::CameraMode mode = runtime::CameraMode::follow;
  double distance = 2.4;            // From the vehicle, m. Overhead, a few times higher.
  Vec3 position{-8.0, -8.0, -3.0};  // CameraMode::fixed only: world frame, m.
};

// What is shown for a simulation snapshot: the ground, the vehicles, everyone's
// drawings, and a camera following one of the vehicles. Shared by the viewer and
// the video recorder, so they show the same thing.
class SceneView {
 public:
  explicit SceneView(const CameraSettings& camera);

  // The meshes to create the renderer with; frames refer to them by index.
  [[nodiscard]] std::span<const render::Mesh> meshes() const { return meshes_; }

  // Builds meshes for the vehicles if they came or went. Returns whether meshes()
  // changed, so that the renderer needs them again. Call with each snapshot's roster
  // before frame().
  [[nodiscard]] bool update_vehicles(const std::shared_ptr<const runtime::Roster>& roster);

  // Rebuilds the drawings' geometry if they changed since the last call. After
  // switching to drawings from another source, call forget_drawings() first.
  void update_drawings(const ipc::VizScene& drawings);
  void forget_drawings();

  [[nodiscard]] runtime::CameraMode camera_mode() const { return camera_.mode; }
  void set_camera_mode(runtime::CameraMode mode) { camera_.mode = mode; }

  // The frame to render for a snapshot, with the camera following vehicle `follow`
  // (or looking at the origin if there is no such vehicle), for an image of the
  // given size. It refers to data in this object, so it stays valid until the next
  // call.
  [[nodiscard]] render::FrameView frame(const runtime::Snapshot& snapshot, std::size_t follow,
                                        std::uint32_t width, std::uint32_t height);

 private:
  CameraSettings camera_;
  std::vector<render::Mesh> meshes_;  // The ground, then each vehicle in roster order.
  std::shared_ptr<const runtime::Roster> roster_;  // The one meshes_ were built for.
  bool have_drawings_ = false;
  std::uint64_t drawings_version_ = 0;
  std::vector<render::Vertex> drawing_vertices_;
  std::vector<render::DrawItem> items_;
};

}  // namespace sim::scene
