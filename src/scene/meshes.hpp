#pragma once

#include "ipc/sim_server.hpp"
#include "render/scene.hpp"
#include "sim/multirotor.hpp"

namespace sim::scene {

// A simple model of the vehicle in its body frame: a body with an orange nose,
// arms, motors, and propeller discs that are orange at the front and blue at the
// back, so the heading is obvious from any angle.
[[nodiscard]] render::Mesh make_multirotor_mesh(const MultirotorModel& model);

// A checkered square in the plane z = 0, centered on the origin.
[[nodiscard]] render::Mesh make_ground_mesh(double tile_size, int tiles_per_side);

// World-space geometry for drawings received from other processes: markers become
// diamonds with a line down to the ground, paths become chains of thin beams.
[[nodiscard]] render::Mesh make_drawings_mesh(const ipc::VizScene& scene);

}  // namespace sim::scene
