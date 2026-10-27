#pragma once

// Wire format for the lockstep connection between the simulator and a
// software-in-the-loop (SITL) flight controller. C23 and C++ compatible.
//
// Transport: a UNIX domain socket of type SOCK_SEQPACKET per vehicle, at
// <socket dir>/vehicles/<vehicle name>.sock. Each send() is one message, so message
// boundaries are preserved. The simulator accepts one flight controller at a time
// per vehicle, and steps once every vehicle's flight controller has answered.
//
// Both ends run on the same machine, so messages are plain structs in native byte
// order; the static_asserts below catch layout differences. Units are SI. Frames:
// world NED (x north, y east, z down), body FRD (x forward, y right, z down).
//
// Sequence:
//   1. On connect, the simulator sends sim_sitl_hello.
//   2. The simulator sends sim_sitl_sensors for the current step and waits.
//   3. The flight controller answers with sim_sitl_actuators, echoing the sequence.
//   4. The simulator applies the commands, advances one step, and continues at 2.
// Simulated time only advances when the flight controller answers, so the two run
// in lockstep at whatever speed the slower allows. If the flight controller
// disconnects, the simulation holds until one connects again.

#include <stddef.h>
#include <stdint.h>

#define SIM_SITL_PROTOCOL_VERSION 1u
#define SIM_SITL_MAX_ROTORS 12u

enum sim_sitl_message_type : uint32_t {
  SIM_SITL_HELLO = 1,
  SIM_SITL_SENSORS = 2,
  SIM_SITL_ACTUATORS = 3,
};

struct sim_sitl_header {
  uint32_t type;     // enum sim_sitl_message_type
  uint32_t version;  // SIM_SITL_PROTOCOL_VERSION
};

struct sim_sitl_rotor {
  double position[3];  // Body frame, m.
  double spin;         // +1 if it turns clockwise seen from above, -1 if counter-clockwise.
};

// Simulator -> flight controller, once per connection. Describes the simulated
// vehicle. A flight controller with its own vehicle configuration may ignore it,
// but must order its motor outputs as the rotors are listed here.
struct sim_sitl_hello {
  struct sim_sitl_header header;
  uint32_t rotor_count;
  uint32_t reserved;
  uint64_t step_ns;           // Simulation step length.
  double mass;                // kg
  double inertia[3];          // Principal moments about body x, y, z, kg m².
  double thrust_coefficient;  // k_T in thrust = k_T ω², N/(rad/s)².
  double torque_coefficient;  // k_Q in drag torque = k_Q ω², N m/(rad/s)².
  double max_rotor_speed;     // ω at motor command 1, rad/s. Command 0.5 means half of it.
  struct sim_sitl_rotor rotors[SIM_SITL_MAX_ROTORS];
};

enum sim_sitl_sensor_flags : uint32_t {
  SIM_SITL_BAROMETER_FRESH = 1u << 0,  // The barometer was sampled at this step.
  SIM_SITL_RESET = 1u << 1,            // The simulation was reset since the last message.
};

struct sim_sitl_ground_truth {
  double position[3];          // World frame, m.
  double velocity[3];          // World frame, m/s.
  double attitude[4];          // Quaternion w, x, y, z rotating body vectors into the world frame.
  double angular_velocity[3];  // Body frame, rad/s.
};

// Simulator -> flight controller, once per step.
struct sim_sitl_sensors {
  struct sim_sitl_header header;
  uint32_t flags;          // enum sim_sitl_sensor_flags
  uint32_t reserved;
  uint64_t sequence;       // One higher each step; echo it in the answer.
  uint64_t time_ns;        // Simulation time of the measurements.
  double accelerometer[3]; // Specific force, body frame, m/s². About (0, 0, -9.81) at rest.
  double gyroscope[3];     // Angular velocity, body frame, rad/s.
  double pressure;         // Barometric pressure, Pa. The latest sample if not fresh.
  double temperature;      // Barometer temperature, °C.
  double pilot[4];         // Roll, pitch, yaw and throttle sticks in [-1, 1].
  // The true state, for logging and evaluating estimators. Not meant for control.
  struct sim_sitl_ground_truth truth;
};

// Flight controller -> simulator, once per sim_sitl_sensors.
struct sim_sitl_actuators {
  struct sim_sitl_header header;
  uint32_t reserved[2];
  uint64_t sequence;                   // From the sim_sitl_sensors being answered.
  double motors[SIM_SITL_MAX_ROTORS];  // Normalized speed commands in [0, 1], in rotor order.
};

static_assert(sizeof(struct sim_sitl_hello) == 464, "unexpected padding in sim_sitl_hello");
static_assert(sizeof(struct sim_sitl_sensors) == 232, "unexpected padding in sim_sitl_sensors");
static_assert(sizeof(struct sim_sitl_actuators) == 120, "unexpected padding in sim_sitl_actuators");
static_assert(offsetof(struct sim_sitl_sensors, truth) == 128, "unexpected padding in sim_sitl_sensors");
