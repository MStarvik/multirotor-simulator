#pragma once

#include <array>

#include "sim/math.hpp"
#include "sim/multirotor.hpp"

namespace sim {

// The true state of the vehicle. Frames as in multirotor.hpp: world NED, body FRD.
struct VehicleState {
  Vec3 position{};          // World frame, m.
  Vec3 velocity{};          // World frame, m/s.
  Vec3 acceleration{};      // World frame, m/s², over the last step (including contact).
  Quat attitude{};          // Rotates body-frame vectors into the world frame.
  Vec3 angular_velocity{};  // Body frame, rad/s.
  std::array<double, max_rotors> rotor_speeds{};  // rad/s
  bool on_ground = true;
};

}  // namespace sim
