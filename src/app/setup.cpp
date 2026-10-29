#include "app/setup.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "ipc/sitl_controller.hpp"

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

runtime::SimRunner make_runner(const runtime::Options& options, const ipc::SocketDirectory& sockets,
                               runtime::ClockPolicy clock) {
  const double step_size = options.step_size;
  auto make_controller = [&sockets, step_size](const runtime::VehicleSpec& spec)
      -> std::unique_ptr<runtime::FlightController> {
    return std::make_unique<ipc::SitlController>(spec.name, spec.model, step_size,
                                                 sockets.vehicle_socket(spec.name));
  };
  return runtime::SimRunner{step_size, options.seed, std::move(make_controller), clock};
}

std::vector<runtime::VehicleSpec> initial_vehicles(const runtime::Options& options) {
  const MultirotorModel model = make_multirotor(options.frame);
  std::vector<runtime::VehicleSpec> vehicles;
  for (std::size_t i = 0; i < options.vehicles.size(); ++i) {
    vehicles.push_back({
        .name = options.vehicles[i],
        .model = model,
        .initial_state = {.position = start_position(model, i)},
    });
  }
  return vehicles;
}

}  // namespace sim::app
