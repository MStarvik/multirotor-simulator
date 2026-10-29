#pragma once

#include <cstddef>
#include <vector>

#include "ipc/socket_directory.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

// Puts together what the programs share: the simulation, the flight controllers it
// makes for its vehicles, and the vehicles the options ask for.

namespace sim::app {

// Where vehicle i starts: on the ground in a row heading east from the origin,
// far enough apart that the propellers are well clear of each other.
[[nodiscard]] Vec3 start_position(const MultirotorModel& model, std::size_t index);

// A runner without vehicles yet. Each vehicle waits for an external flight
// controller on its SITL socket in `sockets`.
[[nodiscard]] runtime::SimRunner make_runner(const runtime::Options& options, const ipc::SocketDirectory& sockets,
                                             runtime::ClockPolicy clock);

// The vehicles the options ask for.
[[nodiscard]] std::vector<runtime::VehicleSpec> initial_vehicles(const runtime::Options& options);

}  // namespace sim::app
