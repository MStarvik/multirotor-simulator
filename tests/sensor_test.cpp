// Checks of the simulated sensors against what real ones would report.

#include <array>
#include <cmath>
#include <cstdint>
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
  sim::Simulation simulation;
  simulation.add_vehicle(sim::make_multirotor({}));
  sim::Vec3 force_sum;
  sim::Vec3 rate_sum;
  double pressure_sum = 0.0;
  int pressure_samples = 0;
  constexpr int steps = 2000;
  for (int i = 0; i < steps; ++i) {
    simulation.step();
    const sim::SensorReadings& readings = simulation.vehicles()[0].sensors();
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
  sim::VehicleState high_up;
  high_up.position.z = -100.0;
  high_up.on_ground = false;
  sim::Simulation simulation;
  simulation.add_vehicle(sim::make_multirotor({}), high_up);
  check(near(simulation.vehicles()[0].sensors().barometer.pressure, 101325.0 - 1201.0, 10.0),
        "barometer reads about 1200 Pa less at 100 m");
}

void test_determinism() {
  // The accelerometer reading of each of two vehicles after a while.
  const auto run = [](std::uint64_t seed) {
    sim::Simulation simulation{0.001, seed};
    simulation.add_vehicle(sim::make_multirotor({}));
    simulation.add_vehicle(sim::make_multirotor({}), {.position = {0.0, 2.0, 0.0}});
    for (int i = 0; i < 100; ++i) {
      simulation.step();
    }
    return std::array{simulation.vehicles()[0].sensors().imu.specific_force.x,
                      simulation.vehicles()[1].sensors().imu.specific_force.x};
  };
  const auto first = run(7);
  check(first == run(7), "the same seed gives the same noise");
  check(first[0] != run(8)[0], "different seeds give different noise");
  check(first[0] != first[1], "vehicles get different noise");
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
