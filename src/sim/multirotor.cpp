#include "sim/multirotor.hpp"

#include <format>
#include <numbers>
#include <stdexcept>

namespace sim {

MultirotorModel make_multirotor(const SymmetricFrame& frame) {
  const int count = frame.rotor_count;
  if (count < 4 || count > static_cast<int>(max_rotors) || count % 2 != 0) {
    throw std::invalid_argument{
        std::format("rotor count must be even and between 4 and {}, got {}", max_rotors, count)};
  }
  if (frame.arm_length <= 0.0 || frame.body.mass <= 0.0 || frame.body.inertia.x <= 0.0 ||
      frame.body.inertia.y <= 0.0 || frame.body.inertia.z <= 0.0) {
    throw std::invalid_argument{"arm length, mass and inertia must be positive"};
  }

  MultirotorModel model{
      .body = frame.body, .propulsion = frame.propulsion, .sensors = frame.sensors, .rotors = {}};
  for (int i = 0; i < count; ++i) {
    // Measured from the nose, clockwise seen from above (from +x towards +y).
    const double angle = 2.0 * std::numbers::pi * (i + 0.5) / count;
    model.rotors.push_back({
        .position = {frame.arm_length * std::cos(angle), frame.arm_length * std::sin(angle),
                     -frame.rotor_height},
        .spin = i % 2 == 0 ? -1.0 : 1.0,
    });
  }
  return model;
}

}  // namespace sim
