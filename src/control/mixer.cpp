#include "control/mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim::control {
namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;

// Gauss-Jordan elimination with partial pivoting.
Matrix4 invert(Matrix4 m) {
  Matrix4 inverse{};
  for (std::size_t i = 0; i < 4; ++i) {
    inverse[i][i] = 1.0;
  }
  for (std::size_t col = 0; col < 4; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 4; ++row) {
      if (std::abs(m[row][col]) > std::abs(m[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(m[pivot][col]) < 1e-12) {
      throw std::invalid_argument{"rotor layout cannot control thrust, roll, pitch and yaw"};
    }
    std::swap(m[col], m[pivot]);
    std::swap(inverse[col], inverse[pivot]);

    const double scale = 1.0 / m[col][col];
    for (std::size_t k = 0; k < 4; ++k) {
      m[col][k] *= scale;
      inverse[col][k] *= scale;
    }
    for (std::size_t row = 0; row < 4; ++row) {
      if (row != col) {
        const double factor = m[row][col];
        for (std::size_t k = 0; k < 4; ++k) {
          m[row][k] -= factor * m[col][k];
          inverse[row][k] -= factor * inverse[col][k];
        }
      }
    }
  }
  return inverse;
}

}  // namespace

Mixer::Mixer(const MultirotorModel& model)
    : rotor_count_{model.rotors.size()}, propulsion_{model.propulsion} {
  // Effectiveness of each rotor's thrust t on (thrust, torque x, torque y, torque z).
  // The force is (0, 0, -t) at the rotor's position, and the reaction torque is
  // -spin * (k_Q / k_T) * t about z.
  const double torque_per_thrust = propulsion_.torque_coefficient / propulsion_.thrust_coefficient;
  std::array<std::array<double, 4>, max_rotors> effectiveness{};
  for (std::size_t i = 0; i < rotor_count_; ++i) {
    const Rotor& rotor = model.rotors[i];
    effectiveness[i] = {1.0, -rotor.position.y, rotor.position.x, -rotor.spin * torque_per_thrust};
  }

  // Pseudo-inverse: allocation = Bᵀ (B Bᵀ)⁻¹, with B the 4×N effectiveness matrix
  // (stored transposed above, one row per rotor).
  Matrix4 gram{};
  for (std::size_t r = 0; r < 4; ++r) {
    for (std::size_t c = 0; c < 4; ++c) {
      for (std::size_t i = 0; i < rotor_count_; ++i) {
        gram[r][c] += effectiveness[i][r] * effectiveness[i][c];
      }
    }
  }
  const Matrix4 gram_inverse = invert(gram);
  for (std::size_t i = 0; i < rotor_count_; ++i) {
    for (std::size_t c = 0; c < 4; ++c) {
      for (std::size_t k = 0; k < 4; ++k) {
        allocation_[i][c] += effectiveness[i][k] * gram_inverse[k][c];
      }
    }
  }
}

MotorCommands Mixer::mix(double thrust, const Vec3& torque) const {
  const std::array wrench{thrust, torque.x, torque.y, torque.z};
  const double max_speed = propulsion_.max_rotor_speed;
  const double max_thrust = propulsion_.thrust_coefficient * max_speed * max_speed;

  MotorCommands commands{};
  for (std::size_t i = 0; i < rotor_count_; ++i) {
    double rotor_thrust = 0.0;
    for (std::size_t k = 0; k < 4; ++k) {
      rotor_thrust += allocation_[i][k] * wrench[k];
    }
    // Simple clipping; a real mixer would trade thrust for attitude authority here.
    rotor_thrust = std::clamp(rotor_thrust, 0.0, max_thrust);
    commands[i] = std::sqrt(rotor_thrust / propulsion_.thrust_coefficient) / max_speed;
  }
  return commands;
}

}  // namespace sim::control
