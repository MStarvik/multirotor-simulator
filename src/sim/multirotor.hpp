#pragma once

#include <cstddef>
#include <vector>

#include "sim/math.hpp"
#include "sim/sensor_params.hpp"

// Frames: the world frame is NED (x north, y east, z down) and the body frame is
// FRD (x forward, y right, z down), as in PX4 and MAVLink.

namespace sim {

inline constexpr std::size_t max_rotors = 12;

struct BodyParams {
  double mass = 1.0;                  // kg
  Vec3 inertia{0.011, 0.011, 0.021};  // Principal moments of inertia about body x, y, z, kg m².
  double linear_drag = 0.1;           // N per m/s, opposing the velocity.
  double angular_drag = 0.003;        // N m per rad/s, opposing the angular velocity.
};

// Shared by all rotors.
struct PropulsionParams {
  double thrust_coefficient = 4.9e-6;  // k_T in thrust = k_T ω², N/(rad/s)².
  double torque_coefficient = 7.8e-8;  // k_Q in drag torque = k_Q ω², N m/(rad/s)².
  double max_rotor_speed = 1000.0;     // rad/s
  double motor_time_constant = 0.03;   // s, first-order response to a speed command.
  double propeller_radius = 0.12;      // m
};

struct Rotor {
  Vec3 position;  // Body frame, m.
  double spin;    // +1 if it turns clockwise seen from above, -1 if counter-clockwise.
};

// Everything the simulation needs to know about a vehicle. Generated from a few
// parameters for now; a model loader (e.g. for URDF) could produce one later.
struct MultirotorModel {
  BodyParams body;
  PropulsionParams propulsion;
  SensorParams sensors;
  std::vector<Rotor> rotors;
};

// A flat frame with identical rotors evenly spaced on a circle.
struct SymmetricFrame {
  int rotor_count = 4;         // Even, so that the spin directions alternate and cancel.
  double arm_length = 0.225;   // Distance from the center to each rotor, m.
  double rotor_height = 0.03;  // Height of the rotors above the center of mass, m.
  BodyParams body;
  PropulsionParams propulsion;
  SensorParams sensors;
};

// Rotor 0 sits just right of the nose (the "X" layout for a quadrotor). The rest
// follow clockwise seen from above, alternating spin direction.
MultirotorModel make_multirotor(const SymmetricFrame& frame);

}  // namespace sim
