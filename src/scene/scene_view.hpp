#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "ipc/viz_server.hpp"
#include "render/scene.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

namespace sim::scene {

// What is shown for a simulation snapshot: the ground, the vehicles, the drawings
// received on the viz socket, and a camera following one of the vehicles. Shared by
// the interactive viewer and the video recorder, so they show the same thing.
class SceneView {
 public:
  // All vehicles share the model.
  SceneView(const MultirotorModel& model, double camera_distance);

  // The meshes to create the renderer with; frames refer to them by index.
  [[nodiscard]] std::span<const render::Mesh> meshes() const { return meshes_; }

  // Rebuilds the drawings' geometry if they changed since the last call.
  void update_drawings(const ipc::VizServer& server);

  // The frame to render for a snapshot, with the camera following vehicle `follow`,
  // for an image of the given size. It refers to data in this object, so it stays
  // valid until the next call.
  [[nodiscard]] render::FrameView frame(const runtime::Snapshot& snapshot, std::size_t follow,
                                        std::uint32_t width, std::uint32_t height);

 private:
  std::array<render::Mesh, 2> meshes_;
  Vec3 camera_offset_;  // From the vehicle to the camera, world frame.
  std::uint64_t drawings_version_ = 0;
  std::vector<render::Vertex> drawing_vertices_;
  std::vector<render::DrawItem> items_;
};

}  // namespace sim::scene
