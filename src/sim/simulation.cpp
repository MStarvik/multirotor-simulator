#include "sim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

double validated_step_size(double step_size) {
  if (!(step_size > 0.0)) {
    throw std::invalid_argument{"step size must be positive"};
  }
  return step_size;
}

}  // namespace

Simulation::Simulation(MultirotorModel model, double step_size, std::uint64_t seed)
    : model_{std::move(model)},
      step_size_{validated_step_size(step_size)},
      seed_{seed},
      sensors_{model_.sensors, step_size_, seed_} {
  if (model_.rotors.empty() || model_.rotors.size() > max_rotors) {
    throw std::invalid_argument{"model must have between 1 and max_rotors rotors"};
  }
  sensors_.sample(state_, steps_);
}

void Simulation::reset(const VehicleState& initial_state) {
  steps_ = 0;
  state_ = initial_state;
  commands_ = {};
  sensors_.reset(seed_);
  sensors_.sample(state_, steps_);
}

void Simulation::step() {
  const double dt = step_size_;
  const BodyParams& body = model_.body;
  const PropulsionParams& propulsion = model_.propulsion;

  // Motors follow their speed command with a first-order lag (discretized exactly),
  // and each rotor's thrust and drag torque scale with its speed squared.
  const double motor_blend = 1.0 - std::exp(-dt / propulsion.motor_time_constant);
  Vec3 force_body;
  Vec3 torque_body;
  for (std::size_t i = 0; i < model_.rotors.size(); ++i) {
    const Rotor& rotor = model_.rotors[i];
    double& speed = state_.rotor_speeds[i];
    const double target_speed = std::clamp(commands_[i], 0.0, 1.0) * propulsion.max_rotor_speed;
    speed += (target_speed - speed) * motor_blend;

    const double thrust = propulsion.thrust_coefficient * speed * speed;
    const Vec3 rotor_force{0.0, 0.0, -thrust};  // Along body up.
    force_body += rotor_force;
    torque_body += cross(rotor.position, rotor_force);
    // The drag on the propeller reacts on the body, against the direction of spin.
    torque_body.z -= rotor.spin * propulsion.torque_coefficient * speed * speed;
  }

  // Translation, in the world frame. Semi-implicit Euler: velocity first.
  const Vec3 force_world = rotate(state_.attitude, force_body) +
                           Vec3{0.0, 0.0, body.mass * gravity} - body.linear_drag * state_.velocity;
  const Vec3 previous_velocity = state_.velocity;
  state_.velocity += force_world * (dt / body.mass);
  state_.position += state_.velocity * dt;

  // Rotation, in the body frame (Euler's rotation equations).
  const Vec3 omega = state_.angular_velocity;
  torque_body -= body.angular_drag * omega;
  const Vec3 angular_acceleration =
      cwise_div(torque_body - cross(omega, cwise_mul(body.inertia, omega)), body.inertia);
  state_.angular_velocity += angular_acceleration * dt;
  state_.attitude =
      normalized(state_.attitude * from_rotation_vector(state_.angular_velocity * dt));

  resolve_ground_contact();
  state_.acceleration = (state_.velocity - previous_velocity) / dt;
  ++steps_;
  sensors_.sample(state_, steps_);
}

// A crude contact model: the ground is the plane z = 0, which the vehicle can't
// sink into, and friction quickly stops it sliding or spinning while it touches.
// Enough for taking off and landing, not for crashes.
void Simulation::resolve_ground_contact() {
  state_.on_ground = state_.position.z >= 0.0;
  if (!state_.on_ground) {
    return;
  }
  state_.position.z = 0.0;
  state_.velocity.z = std::min(state_.velocity.z, 0.0);

  constexpr double friction_time_constant = 0.05;  // s
  const double damping = std::exp(-step_size_ / friction_time_constant);
  state_.velocity.x *= damping;
  state_.velocity.y *= damping;
  state_.angular_velocity *= damping;
}

}  // namespace sim
