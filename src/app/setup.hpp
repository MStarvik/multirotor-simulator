#pragma once

#include <cstddef>

#include "ipc/socket_directory.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/multirotor.hpp"

// Puts together what the programs share: the vehicles from the options, and a flight
// controller for each.

namespace sim::app {

// Where vehicle i starts: on the ground in a row heading east from the origin,
// far enough apart that the propellers are well clear of each other.
[[nodiscard]] Vec3 start_position(const MultirotorModel& model, std::size_t index);

// A runner with the options' vehicles, each with its own flight controller: the
// built-in one, or with --sitl an external one on its socket in `sockets`.
[[nodiscard]] runtime::SimRunner make_runner(const runtime::Options& options, const MultirotorModel& model,
                                             const ipc::SocketDirectory* sockets, runtime::ClockPolicy clock);

}  // namespace sim::app
