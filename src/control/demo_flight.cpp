#include "control/demo_flight.hpp"

#include <array>
#include <stop_token>

namespace sim::control {
namespace {

struct ScriptStep {
  double start;  // Simulation time, s.
  runtime::PilotInput input;
};

constexpr std::array script{
    ScriptStep{.start = 0.0, .input = {.throttle = 0.5}},
    ScriptStep{.start = 3.0, .input = {}},
    ScriptStep{.start = 5.0, .input = {.pitch = 0.3}},
    ScriptStep{.start = 7.0, .input = {}},
};

}  // namespace

void DemoFlight::run_until(runtime::SimRunner& runner, double until) {
  // Stop at each step's start, so its input applies from exactly then.
  while (next_step_ < script.size() && script[next_step_].start < until) {
    runner.run(std::stop_token{}, script[next_step_].start);
    for (std::size_t vehicle = 0; vehicle < runner.vehicle_count(); ++vehicle) {
      runner.push(runtime::SetPilotInput{.vehicle = vehicle, .input = script[next_step_].input});
    }
    ++next_step_;
  }
  runner.run(std::stop_token{}, until);
}

}  // namespace sim::control
