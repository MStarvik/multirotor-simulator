#include "viewer/transform.hpp"

#include <cmath>
#include <cstddef>

namespace sim::viewer {
namespace {

// Builds a column-major matrix from values written row by row, which reads naturally.
Mat4 from_rows(double m00, double m01, double m02, double m03,  //
               double m10, double m11, double m12, double m13,  //
               double m20, double m21, double m22, double m23,  //
               double m30, double m31, double m32, double m33) {
  const auto f = [](double v) { return static_cast<float>(v); };
  return {f(m00), f(m10), f(m20), f(m30),  //
          f(m01), f(m11), f(m21), f(m31),  //
          f(m02), f(m12), f(m22), f(m32),  //
          f(m03), f(m13), f(m23), f(m33)};
}

}  // namespace

Mat4 multiply(const Mat4& a, const Mat4& b) {
  Mat4 result{};
  for (std::size_t col = 0; col < 4; ++col) {
    for (std::size_t row = 0; row < 4; ++row) {
      float sum = 0.0f;
      for (std::size_t k = 0; k < 4; ++k) {
        sum += a[k * 4 + row] * b[col * 4 + k];
      }
      result[col * 4 + row] = sum;
    }
  }
  return result;
}

Mat4 translation(const Vec3& offset) { return rigid_transform(offset, Quat{}); }

Mat4 rigid_transform(const Vec3& position, const Quat& attitude) {
  // The columns of the rotation matrix are the rotated basis vectors.
  const Vec3 x = rotate(attitude, {1.0, 0.0, 0.0});
  const Vec3 y = rotate(attitude, {0.0, 1.0, 0.0});
  const Vec3 z = rotate(attitude, {0.0, 0.0, 1.0});
  return from_rows(x.x, y.x, z.x, position.x,  //
                   x.y, y.y, z.y, position.y,  //
                   x.z, y.z, z.z, position.z,  //
                   0.0, 0.0, 0.0, 1.0);
}

Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Vec3 forward = normalized(target - eye);
  const Vec3 right = normalized(cross(forward, up));
  const Vec3 camera_up = cross(right, forward);
  return from_rows(right.x, right.y, right.z, -dot(right, eye),               //
                   camera_up.x, camera_up.y, camera_up.z, -dot(camera_up, eye),  //
                   -forward.x, -forward.y, -forward.z, dot(forward, eye),     //
                   0.0, 0.0, 0.0, 1.0);
}

Mat4 perspective(double vertical_fov, double aspect, double near_plane, double far_plane) {
  const double focal = 1.0 / std::tan(0.5 * vertical_fov);
  const double depth_scale = far_plane / (near_plane - far_plane);
  return from_rows(focal / aspect, 0.0, 0.0, 0.0,  //
                   0.0, -focal, 0.0, 0.0,           // Vulkan's clip-space y points down.
                   0.0, 0.0, depth_scale, near_plane * depth_scale,  //
                   0.0, 0.0, -1.0, 0.0);
}

}  // namespace sim::viewer
