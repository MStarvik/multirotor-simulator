// Checks of the simulated sensors against what real ones would report.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <print>
#include <source_location>

#include "sim/multirotor.hpp"
#include "sim/simulation.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* what, std::source_location where = std::source_location::current()) {
  if (!condition) {
    std::println(stderr, "{}:{}: check failed: {}", where.file_name(), where.line(), what);
    ++failures;
  }
}

bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

// Averages the sensors over many steps of a vehicle sitting on the ground.
void test_at_rest() {
  sim::Simulation simulation{sim::make_multirotor({})};
  sim::Vec3 force_sum;
  sim::Vec3 rate_sum;
  double pressure_sum = 0.0;
  int pressure_samples = 0;
  constexpr int steps = 2000;
  for (int i = 0; i < steps; ++i) {
    simulation.step();
    const sim::SensorReadings& readings = simulation.sensors();
    force_sum += readings.imu.specific_force;
    rate_sum += readings.imu.angular_velocity;
    if (readings.barometer_fresh) {
      pressure_sum += readings.barometer.pressure;
      ++pressure_samples;
    }
  }
  const sim::Vec3 force = force_sum / steps;
  const sim::Vec3 rate = rate_sum / steps;

  // An accelerometer at rest measures the reaction to gravity: up, i.e. -z in FRD.
  check(near(force.x, 0.0, 0.05) && near(force.y, 0.0, 0.05), "accelerometer reads no horizontal force");
  check(near(force.z, -sim::gravity, 0.05), "accelerometer reads -g along body z at rest");
  check(near(rate.x, 0.0, 0.01) && near(rate.y, 0.0, 0.01) && near(rate.z, 0.0, 0.01),
        "gyroscope reads no rotation at rest");
  check(pressure_samples == 100, "barometer samples at 50 Hz");
  check(near(pressure_sum / pressure_samples, 101325.0, 5.0), "barometer reads sea-level pressure");
}

// Pressure drops by about 12 Pa per meter near sea level.
void test_barometer_altitude() {
  sim::Simulation simulation{sim::make_multirotor({})};
  sim::VehicleState high_up;
  high_up.position.z = -100.0;
  high_up.on_ground = false;
  simulation.reset(high_up);
  check(near(simulation.sensors().barometer.pressure, 101325.0 - 1201.0, 10.0),
        "barometer reads about 1200 Pa less at 100 m");
}

void test_determinism() {
  const auto run = [](std::uint64_t seed) {
    sim::Simulation simulation{sim::make_multirotor({}), 0.001, seed};
    for (int i = 0; i < 100; ++i) {
      simulation.step();
    }
    return simulation.sensors().imu.specific_force.x;
  };
  check(run(7) == run(7), "the same seed gives the same noise");
  check(run(7) != run(8), "different seeds give different noise");
}

}  // namespace

int main() {
  test_at_rest();
  test_barometer_altitude();
  test_determinism();
  if (failures == 0) {
    std::println("All sensor checks passed");
  }
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
