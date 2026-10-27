#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "ipc/sim_server.hpp"
#include "render/scene.hpp"
#include "runtime/sim_runner.hpp"

namespace sim::scene {

// What is shown for a simulation snapshot: the ground, the vehicles, the drawings
// received on the viz socket, and a camera following one of the vehicles. Shared by
// the interactive viewer and the video recorder, so they show the same thing.
class SceneView {
 public:
  explicit SceneView(double camera_distance);

  // The meshes to create the renderer with; frames refer to them by index.
  [[nodiscard]] std::span<const render::Mesh> meshes() const { return meshes_; }

  // Builds meshes for the vehicles if they came or went. Returns whether meshes()
  // changed, so that the renderer needs them again. Call with each snapshot's roster
  // before frame().
  [[nodiscard]] bool update_vehicles(const runtime::Roster& roster);

  // Rebuilds the drawings' geometry if they changed since the last call.
  void update_drawings(const ipc::SimServer& server);

  // The frame to render for a snapshot, with the camera following vehicle `follow`
  // (or looking at the origin if there is no such vehicle), for an image of the
  // given size. It refers to data in this object, so it stays
  // valid until the next call.
  [[nodiscard]] render::FrameView frame(const runtime::Snapshot& snapshot, std::size_t follow,
                                        std::uint32_t width, std::uint32_t height);

 private:
  std::vector<render::Mesh> meshes_;  // The ground, then each vehicle in roster order.
  std::uint64_t roster_version_ = 0;
  Vec3 camera_offset_;  // From the vehicle to the camera, world frame.
  std::uint64_t drawings_version_ = 0;
  std::vector<render::Vertex> drawing_vertices_;
  std::vector<render::DrawItem> items_;
};

}  // namespace sim::scene
