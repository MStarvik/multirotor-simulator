#include "viewer/meshes.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <variant>

namespace sim::viewer {
namespace {

using render::Color;
using render::Mesh;

constexpr Color body_color{0.12f, 0.12f, 0.14f};
constexpr Color arm_color{0.35f, 0.35f, 0.38f};
constexpr Color front_color{0.95f, 0.35f, 0.05f};
constexpr Color back_color{0.10f, 0.35f, 0.85f};
constexpr Color side_color{0.75f, 0.75f, 0.75f};
constexpr std::array ground_colors{Color{0.20f, 0.26f, 0.18f}, Color{0.30f, 0.37f, 0.26f}};

std::array<float, 3> to_float(const Vec3& v) {
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

void add_triangle(Mesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& normal,
                  const Color& color) {
  for (const Vec3& corner : {a, b, c}) {
    mesh.vertices.push_back({.position = to_float(corner), .normal = to_float(normal), .color = color});
  }
}

void add_quad(Mesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d,
              const Vec3& normal, const Color& color) {
  add_triangle(mesh, a, b, c, normal, color);
  add_triangle(mesh, a, c, d, normal, color);
}

// A box with the given half extents, rotated by yaw about z and moved to center.
void add_box(Mesh& mesh, const Vec3& center, const Vec3& half_extents, double yaw, const Color& color) {
  const Quat rotation = from_euler({.yaw = yaw});
  const auto place = [&](const Vec3& local) { return center + rotate(rotation, local); };

  struct Face {
    Vec3 normal;
    Vec3 u;
    Vec3 v;
  };
  constexpr std::array faces{
      Face{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Face{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Face{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}}, Face{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
      Face{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}, Face{{0, 0, -1}, {1, 0, 0}, {0, 1, 0}},
  };
  for (const Face& face : faces) {
    const Vec3 n = cwise_mul(face.normal, half_extents);
    const Vec3 u = cwise_mul(face.u, half_extents);
    const Vec3 v = cwise_mul(face.v, half_extents);
    add_quad(mesh, place(n - u - v), place(n + u - v), place(n + u + v), place(n - u + v),
             rotate(rotation, face.normal), color);
  }
}

// A flat disc in a plane of constant z.
void add_disc(Mesh& mesh, const Vec3& center, double radius, const Color& color) {
  constexpr int segments = 32;
  const Vec3 up{0.0, 0.0, -1.0};
  for (int i = 0; i < segments; ++i) {
    const double a0 = 2.0 * std::numbers::pi * i / segments;
    const double a1 = 2.0 * std::numbers::pi * (i + 1) / segments;
    add_triangle(mesh, center, center + radius * Vec3{std::cos(a0), std::sin(a0), 0.0},
                 center + radius * Vec3{std::cos(a1), std::sin(a1), 0.0}, up, color);
  }
}

// A square beam from a to b, without end caps.
void add_beam(Mesh& mesh, const Vec3& a, const Vec3& b, double width, const Color& color) {
  const Vec3 along = b - a;
  const double length = norm(along);
  if (length < 1e-9) {
    return;
  }
  const Vec3 axis = along / length;
  // Any vector not parallel to the axis gives a perpendicular pair.
  const Vec3 helper = std::abs(axis.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
  const Vec3 u = normalized(cross(axis, helper));
  const Vec3 v = cross(axis, u);
  const double half = 0.5 * width;
  for (const auto& [side, offset] : {std::pair{u, v}, std::pair{v, -u}, std::pair{-u, -v}, std::pair{-v, u}}) {
    const Vec3 edge0 = half * (side - offset);
    const Vec3 edge1 = half * (side + offset);
    add_quad(mesh, a + edge0, b + edge0, b + edge1, a + edge1, side, color);
  }
}

// An octahedron: the six points at the given radius along each axis.
void add_diamond(Mesh& mesh, const Vec3& center, double radius, const Color& color) {
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      for (const double sz : {-1.0, 1.0}) {
        const Vec3 x{sx * radius, 0.0, 0.0};
        const Vec3 y{0.0, sy * radius, 0.0};
        const Vec3 z{0.0, 0.0, sz * radius};
        add_triangle(mesh, center + x, center + y, center + z, normalized(Vec3{sx, sy, sz}), color);
      }
    }
  }
}

Vec3 to_vec3(const ipc::Float3& v) { return {v[0], v[1], v[2]}; }

}  // namespace

Mesh make_multirotor_mesh(const MultirotorModel& model) {
  Mesh mesh;
  add_box(mesh, {}, {0.07, 0.045, 0.025}, 0.0, body_color);
  add_box(mesh, {0.085, 0.0, 0.0}, {0.015, 0.02, 0.012}, 0.0, front_color);  // Nose.

  for (const Rotor& rotor : model.rotors) {
    const Vec3& p = rotor.position;
    const double reach = std::hypot(p.x, p.y);
    add_box(mesh, {0.5 * p.x, 0.5 * p.y, 0.5 * p.z}, {0.5 * reach, 0.008, 0.008}, std::atan2(p.y, p.x),
            arm_color);
    add_box(mesh, p, {0.014, 0.014, 0.015}, 0.0, body_color);  // Motor.

    constexpr double side_tolerance = 1e-6;
    const Color& disc_color = p.x > side_tolerance    ? front_color
                              : p.x < -side_tolerance ? back_color
                                                      : side_color;
    // Just above the motor (up is -z).
    add_disc(mesh, p + Vec3{0.0, 0.0, -0.017}, model.propulsion.propeller_radius, disc_color);
  }
  return mesh;
}

Mesh make_ground_mesh(double tile_size, int tiles_per_side) {
  Mesh mesh;
  const Vec3 up{0.0, 0.0, -1.0};
  const double origin = -0.5 * tile_size * tiles_per_side;
  for (int i = 0; i < tiles_per_side; ++i) {
    for (int j = 0; j < tiles_per_side; ++j) {
      const double x = origin + tile_size * i;
      const double y = origin + tile_size * j;
      add_quad(mesh, {x, y, 0.0}, {x + tile_size, y, 0.0}, {x + tile_size, y + tile_size, 0.0},
               {x, y + tile_size, 0.0}, up, ground_colors[static_cast<std::size_t>((i + j) % 2)]);
    }
  }
  return mesh;
}

Mesh make_drawings_mesh(const ipc::VizScene& scene) {
  Mesh mesh;
  for (const auto& [id, drawing] : scene.drawings) {
    if (const auto* marker = std::get_if<ipc::VizMarker>(&drawing)) {
      const Vec3 position = to_vec3(marker->position);
      add_diamond(mesh, position, 0.5 * marker->size, marker->color);
      if (position.z < 0.0) {  // Above the ground (up is -z).
        add_beam(mesh, position, {position.x, position.y, 0.0}, 0.1 * marker->size, marker->color);
      }
    } else if (const auto* path = std::get_if<ipc::VizPath>(&drawing)) {
      for (std::size_t i = 1; i < path->points.size(); ++i) {
        add_beam(mesh, to_vec3(path->points[i - 1]), to_vec3(path->points[i]), path->width, path->color);
      }
    }
  }
  return mesh;
}

}  // namespace sim::viewer
