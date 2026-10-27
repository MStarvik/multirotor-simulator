#pragma once

#include <cstddef>

#include "runtime/sim_runner.hpp"

namespace sim::control {

// A short scripted flight of stick inputs: take off, hover, fly forward, hover.
// Every vehicle flies it. Used by the headless program and the recorder when no
// pilot is around.
class DemoFlight {
 public:
  // Runs the simulation until `until` (simulated seconds), applying each scripted
  // stick input at its time. Call with increasing times.
  void run_until(runtime::SimRunner& runner, double until);

 private:
  std::size_t next_step_ = 0;
};

}  // namespace sim::control
