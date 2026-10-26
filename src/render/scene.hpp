#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

// What the renderer draws. Deliberately independent of the simulation, so the
// renderer can be reused for anything that produces meshes and transforms.

namespace sim::render {

using Mat4 = std::array<float, 16>;  // Column-major, as GLSL expects.
using Color = std::array<float, 3>;  // Linear RGB.

struct Vertex {
  std::array<float, 3> position;
  std::array<float, 3> normal;
  Color color;
};

// A triangle list. Faces are drawn from both sides.
struct Mesh {
  std::vector<Vertex> vertices;
};

struct DrawItem {
  std::size_t mesh;  // Index into the meshes the renderer was created with.
  Mat4 model;        // Model to world transform.
};

struct FrameView {
  Mat4 view_projection;  // World to clip space, for Vulkan's clip conventions.
  std::array<float, 4> clear_color;
  std::span<const DrawItem> items;
  // Triangles already in world space that change from frame to frame, such as
  // drawings from other processes. Uploaded every frame.
  std::span<const Vertex> world_vertices;
};

}  // namespace sim::render
