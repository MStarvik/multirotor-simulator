#pragma once

#include <algorithm>
#include <cmath>

// Small vector and quaternion types for the simulation. Double precision, since
// errors accumulate over many integration steps.

namespace sim {

inline constexpr double gravity = 9.80665;  // m/s²

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Vec3& operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  constexpr Vec3& operator-=(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  constexpr Vec3& operator*=(double scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }
};

constexpr Vec3 operator+(Vec3 a, const Vec3& b) { return a += b; }
constexpr Vec3 operator-(Vec3 a, const Vec3& b) { return a -= b; }
constexpr Vec3 operator-(const Vec3& v) { return {-v.x, -v.y, -v.z}; }
constexpr Vec3 operator*(Vec3 v, double scale) { return v *= scale; }
constexpr Vec3 operator*(double scale, Vec3 v) { return v *= scale; }
constexpr Vec3 operator/(const Vec3& v, double divisor) { return {v.x / divisor, v.y / divisor, v.z / divisor}; }

constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) { return v / norm(v); }

// Component-wise operations, e.g. for applying a diagonal inertia tensor.
constexpr Vec3 cwise_mul(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
constexpr Vec3 cwise_div(const Vec3& a, const Vec3& b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }

inline bool is_finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// A unit quaternion representing a rotation (Hamilton convention).
struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr Quat operator*(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
constexpr Quat operator-(const Quat& q) { return {-q.w, -q.x, -q.y, -q.z}; }
constexpr Quat conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quat normalized(const Quat& q) {
  const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return {q.w / length, q.x / length, q.y / length, q.z / length};
}

// Rotates v by q.
constexpr Vec3 rotate(const Quat& q, const Vec3& v) {
  const Vec3 axis{q.x, q.y, q.z};
  const Vec3 t = 2.0 * cross(axis, v);
  return v + q.w * t + cross(axis, t);
}

// The rotation by |r| radians about the axis r.
inline Quat from_rotation_vector(const Vec3& r) {
  const double angle = norm(r);
  if (angle < 1e-12) {
    return normalized(Quat{1.0, 0.5 * r.x, 0.5 * r.y, 0.5 * r.z});
  }
  const double s = std::sin(0.5 * angle) / angle;
  return {std::cos(0.5 * angle), s * r.x, s * r.y, s * r.z};
}

struct Euler {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// Z-Y-X (yaw, then pitch, then roll) Euler angles, the aerospace convention.
inline Quat from_euler(const Euler& e) {
  const double cr = std::cos(0.5 * e.roll);
  const double sr = std::sin(0.5 * e.roll);
  const double cp = std::cos(0.5 * e.pitch);
  const double sp = std::sin(0.5 * e.pitch);
  const double cy = std::cos(0.5 * e.yaw);
  const double sy = std::sin(0.5 * e.yaw);
  return {cr * cp * cy + sr * sp * sy,
          sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}

inline Euler to_euler(const Quat& q) {
  return {
      .roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
      .pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)),
      .yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
  };
}

inline bool is_finite(const Quat& q) {
  return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

}  // namespace sim
