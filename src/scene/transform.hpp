#pragma once

#include "render/scene.hpp"
#include "sim/math.hpp"

// 4×4 transforms for the renderer, built from the simulation's double-precision
// types. All matrices are column-major.

namespace sim::scene {

using render::Mat4;

[[nodiscard]] Mat4 multiply(const Mat4& a, const Mat4& b);

[[nodiscard]] Mat4 translation(const Vec3& offset);

// Rotation followed by translation, e.g. body to world.
[[nodiscard]] Mat4 rigid_transform(const Vec3& position, const Quat& attitude);

// A right-handed view matrix looking from eye towards target.
[[nodiscard]] Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up);

// Perspective projection for Vulkan: depth in [0, 1] and clip-space y pointing down.
[[nodiscard]] Mat4 perspective(double vertical_fov, double aspect, double near_plane, double far_plane);

}  // namespace sim::scene
