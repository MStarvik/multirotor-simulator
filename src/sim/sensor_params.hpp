#pragma once

namespace sim {

// Noise is given as continuous-time densities, as in datasheets and the usual
// (Kalibr, rotors_simulator) IMU noise model, and discretized for the step size.
// The defaults are those used for the ADIS16448 in rotors_simulator and PX4's
// Gazebo models.
struct ImuParams {
  double accel_noise_density = 0.00186;      // m/s²/√Hz
  double accel_bias_random_walk = 0.006;     // m/s³/√Hz
  double gyro_noise_density = 0.00018665;    // rad/s/√Hz
  double gyro_bias_random_walk = 3.8785e-5;  // rad/s²/√Hz
};

struct BarometerParams {
  double rate = 50.0;               // Hz
  double noise_stddev = 1.5;        // Pa, white noise per sample.
  double drift_random_walk = 0.2;   // Pa/√s, slowly wandering offset.
  double ground_altitude = 0.0;     // m above sea level at z = 0.
};

struct SensorParams {
  ImuParams imu;
  BarometerParams barometer;
};

}  // namespace sim
