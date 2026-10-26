#include "sim/sensors.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// International Standard Atmosphere, troposphere.
constexpr double sea_level_pressure = 101325.0;    // Pa
constexpr double sea_level_temperature = 288.15;   // K
constexpr double temperature_lapse_rate = 0.0065;  // K/m
constexpr double pressure_exponent = 5.25588;      // g M / (R L)

}  // namespace

Sensors::Sensors(const SensorParams& params, double step_size, std::uint64_t seed)
    : params_{params},
      step_size_{step_size},
      barometer_decimation_{static_cast<std::uint64_t>(
          std::max(1.0, std::round(1.0 / (params.barometer.rate * step_size))))} {
  reset(seed);
}

void Sensors::reset(std::uint64_t seed) {
  rng_.seed(seed);
  standard_normal_.reset();
  accel_bias_ = {};
  gyro_bias_ = {};
  barometer_drift_ = 0.0;
  readings_ = {};
}

void Sensors::sample(const VehicleState& state, std::uint64_t step) {
  const double dt = step_size_;
  const ImuParams& imu = params_.imu;

  // Discretization: a bias random walk grows by density·√dt per step, while white
  // noise sampled at 1/dt has standard deviation density/√dt.
  accel_bias_ += gaussian_vec(imu.accel_bias_random_walk * std::sqrt(dt));
  gyro_bias_ += gaussian_vec(imu.gyro_bias_random_walk * std::sqrt(dt));

  // An accelerometer measures acceleration minus gravity, in the body frame.
  const Vec3 specific_force =
      rotate(conjugate(state.attitude), state.acceleration - Vec3{0.0, 0.0, gravity});
  readings_.imu = {
      .specific_force = specific_force + accel_bias_ + gaussian_vec(imu.accel_noise_density / std::sqrt(dt)),
      .angular_velocity =
          state.angular_velocity + gyro_bias_ + gaussian_vec(imu.gyro_noise_density / std::sqrt(dt)),
  };

  readings_.barometer_fresh = step % barometer_decimation_ == 0;
  if (readings_.barometer_fresh) {
    const BarometerParams& barometer = params_.barometer;
    const double sample_interval = static_cast<double>(barometer_decimation_) * dt;
    barometer_drift_ += gaussian(barometer.drift_random_walk * std::sqrt(sample_interval));

    const double altitude = barometer.ground_altitude - state.position.z;
    const double temperature = sea_level_temperature - temperature_lapse_rate * altitude;
    const double pressure =
        sea_level_pressure * std::pow(temperature / sea_level_temperature, pressure_exponent);
    readings_.barometer = {
        .pressure = pressure + barometer_drift_ + gaussian(barometer.noise_stddev),
        .temperature = temperature - 273.15,
    };
  }
}

}  // namespace sim
