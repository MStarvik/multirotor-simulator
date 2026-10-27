#include "app/setup.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control/attitude_controller.hpp"
#include "ipc/sitl_controller.hpp"
#include "sim/simulation.hpp"

namespace sim::app {
namespace {

constexpr double vehicle_gap = 1.5;  // m between the propeller tips of neighbors.

}  // namespace

Vec3 start_position(const MultirotorModel& model, std::size_t index) {
  double radius = 0.0;  // Of the circle around the propellers.
  for (const Rotor& rotor : model.rotors) {
    radius = std::max(radius, std::hypot(rotor.position.x, rotor.position.y) + model.propulsion.propeller_radius);
  }
  return {0.0, static_cast<double>(index) * (2.0 * radius + vehicle_gap), 0.0};
}

runtime::SimRunner make_runner(const runtime::Options& options, const MultirotorModel& model,
                               const ipc::SocketDirectory* sockets, runtime::ClockPolicy clock) {
  if (options.sitl && sockets == nullptr) {
    throw std::logic_error{"SITL needs a socket directory"};
  }
  Simulation simulation{options.step_size, options.seed};
  std::vector<std::unique_ptr<runtime::FlightController>> controllers;
  for (std::size_t i = 0; i < options.vehicles.size(); ++i) {
    simulation.add_vehicle(model, {.position = start_position(model, i)});
    const std::string& name = options.vehicles[i];
    if (options.sitl) {
      controllers.push_back(std::make_unique<ipc::SitlController>(name, model, options.step_size,
                                                                  sockets->vehicle_socket(name)));
    } else {
      controllers.push_back(std::make_unique<control::AttitudeController>(model));
    }
  }
  return runtime::SimRunner{std::move(simulation), std::move(controllers), clock};
}

}  // namespace sim::app
