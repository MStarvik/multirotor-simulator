#pragma once

#include <array>
#include <cstddef>

#include "sim/math.hpp"
#include "sim/multirotor.hpp"
#include "sim/simulation.hpp"

namespace sim::control {

// Distributes a desired collective thrust and body torque over the rotors.
//
// The allocation is the pseudo-inverse of the matrix mapping rotor thrusts to
// thrust and torque, derived from the rotor layout, so it works for any frame.
class Mixer {
 public:
  explicit Mixer(const MultirotorModel& model);

  // thrust: N along body up. torque: N m in the body frame.
  [[nodiscard]] MotorCommands mix(double thrust, const Vec3& torque) const;

 private:
  std::size_t rotor_count_;
  PropulsionParams propulsion_;
  // Row i maps (thrust, torque x, torque y, torque z) to rotor i's thrust.
  std::array<std::array<double, 4>, max_rotors> allocation_{};
};

}  // namespace sim::control
