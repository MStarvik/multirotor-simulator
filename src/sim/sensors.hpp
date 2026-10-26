#pragma once

#include <cstdint>
#include <random>

#include "sim/math.hpp"
#include "sim/sensor_params.hpp"
#include "sim/vehicle_state.hpp"

namespace sim {

struct ImuSample {
  Vec3 specific_force;    // Body frame, m/s². Reads (0, 0, -g) when level and at rest.
  Vec3 angular_velocity;  // Body frame, rad/s.
};

struct BarometerSample {
  double pressure = 0.0;     // Pa
  double temperature = 0.0;  // °C
};

struct SensorReadings {
  ImuSample imu;  // Sampled every step.
  BarometerSample barometer;
  bool barometer_fresh = false;  // Whether the barometer was sampled in the latest step.
};

// Samples the vehicle's sensors after each simulation step. Noise comes from a
// seeded generator, so a run with the same seed and inputs is reproducible.
class Sensors {
 public:
  Sensors(const SensorParams& params, double step_size, std::uint64_t seed);

  void reset(std::uint64_t seed);
  void sample(const VehicleState& state, std::uint64_t step);

  [[nodiscard]] const SensorReadings& readings() const { return readings_; }

 private:
  double gaussian(double stddev) { return stddev * standard_normal_(rng_); }
  Vec3 gaussian_vec(double stddev) { return {gaussian(stddev), gaussian(stddev), gaussian(stddev)}; }

  SensorParams params_;
  double step_size_;
  std::uint64_t barometer_decimation_;  // Steps per barometer sample.

  std::mt19937_64 rng_;
  std::normal_distribution<double> standard_normal_;
  Vec3 accel_bias_;
  Vec3 gyro_bias_;
  double barometer_drift_ = 0.0;
  SensorReadings readings_;
};

}  // namespace sim
