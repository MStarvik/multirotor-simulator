#include "runtime/sim_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <stdexcept>
#include <thread>
#include <utility>

namespace sim::runtime {
namespace {

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

// How far real-time pacing may fall behind before the backlog is dropped. Without a
// limit, a simulation that can't keep up would try ever harder to catch up.
constexpr auto max_lag = std::chrono::milliseconds{100};
constexpr auto rate_interval = std::chrono::milliseconds{500};
constexpr auto pause_poll_interval = std::chrono::milliseconds{10};

// Overload set for std::visit.
template <typename... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};

}  // namespace

bool is_valid_name(std::string_view name) {
  const auto allowed = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
           c == '_' || c == '.';
  };
  return !name.empty() && name.size() <= max_name_length && name.front() != '.' &&
         std::ranges::all_of(name, allowed);
}

SimRunner::SimRunner(double step_size, std::uint64_t seed, ControllerFactory make_controller,
                     ClockPolicy clock)
    : simulation_{step_size, seed},
      make_controller_{std::move(make_controller)},
      roster_{std::make_shared<const Roster>()},
      obstacles_{std::make_shared<const Obstacles>()},
      clock_{clock} {
  publish();
}

void SimRunner::add_vehicle(VehicleSpec spec) {
  add(std::move(spec));
  publish();
}

void SimRunner::add(VehicleSpec spec) {
  if (!is_valid_name(spec.name)) {
    throw std::invalid_argument{std::format(
        "invalid vehicle name '{}'; use up to {} letters, digits, '-', '_' and '.', not starting with '.'",
        spec.name, max_name_length)};
  }
  if (find_vehicle(spec.name)) {
    throw std::invalid_argument{std::format("there already is a vehicle named '{}'", spec.name)};
  }
  if (vehicles_.size() >= max_vehicles) {
    throw std::invalid_argument{std::format("there can be at most {} vehicles", max_vehicles)};
  }
  std::unique_ptr<FlightController> controller = make_controller_(spec);
  simulation_.add_vehicle(spec.model, spec.initial_state);
  vehicles_.push_back({.controller = std::move(controller), .answer = std::nullopt});
  change_roster([&](std::vector<VehicleSpec>& vehicles) { vehicles.push_back(std::move(spec)); });
}

void SimRunner::remove(std::size_t index) {
  const auto offset = static_cast<std::ptrdiff_t>(index);
  simulation_.remove_vehicle(index);
  vehicles_.erase(vehicles_.begin() + offset);  // Closes an external controller's socket.
  change_roster([&](std::vector<VehicleSpec>& vehicles) { vehicles.erase(vehicles.begin() + offset); });
}

void SimRunner::change_roster(const std::function<void(std::vector<VehicleSpec>&)>& change) {
  auto roster = std::make_shared<Roster>(*roster_);
  ++roster->version;
  change(roster->vehicles);
  roster_ = std::move(roster);
  next_snapshot_.vehicles.resize(vehicles_.size());
}

TripleBuffer<Snapshot>& SimRunner::add_snapshot_reader() {
  auto& reader = *other_readers_.emplace_back(std::make_unique<TripleBuffer<Snapshot>>());
  reader.publish(next_snapshot_);
  return reader;
}

void SimRunner::push(Command command, Completion done) {
  const std::lock_guard lock{commands_mutex_};
  pending_commands_.emplace_back(std::move(command), std::move(done));
}

void SimRunner::run(std::stop_token stop, double until) {
  // Real-time pacing: the simulation should be anchor_sim_time + elapsed * speed.
  auto anchor_wall_time = Clock::now();
  double anchor_sim_time = simulation_.time();
  auto rate_wall_time = anchor_wall_time;
  double rate_sim_time = anchor_sim_time;

  while (!stop.stop_requested() && simulation_.time() < until) {
    apply_commands();
    if (std::exchange(repace_, false)) {
      // Real-time pacing restarts from here rather than catching up.
      anchor_wall_time = rate_wall_time = Clock::now();
      anchor_sim_time = rate_sim_time = simulation_.time();
    }

    // Unpaced, nothing would hold a simulation without vehicles.
    const bool idle = paused_ || (vehicles_.empty() && std::holds_alternative<AsFastAsPossible>(clock_));
    const bool stepped = !idle && step();
    waiting_for_controllers_ = !idle && !stepped;
    if (!stepped) {
      // Idle, or a flight controller is not ready (it waits briefly itself).
      real_time_factor_ = 0.0;
      repace_ = true;
      publish();
      if (idle) {
        std::this_thread::sleep_for(pause_poll_interval);
      }
      continue;
    }
    publish();

    const auto now = Clock::now();
    if (now - rate_wall_time >= rate_interval) {
      real_time_factor_ =
          (simulation_.time() - rate_sim_time) / Seconds{now - rate_wall_time}.count();
      rate_wall_time = now;
      rate_sim_time = simulation_.time();
    }

    if (const auto* real_time = std::get_if<RealTime>(&clock_)) {
      const auto target = anchor_wall_time + std::chrono::duration_cast<Clock::duration>(
                                                 Seconds{(simulation_.time() - anchor_sim_time) / real_time->speed});
      if (target > now) {
        std::this_thread::sleep_until(target);
      } else if (now - target > max_lag) {
        // Can't keep up: run slower than requested rather than spiralling.
        anchor_wall_time = now;
        anchor_sim_time = simulation_.time();
      }
    }
  }
}

void SimRunner::apply_commands() {
  {
    const std::lock_guard lock{commands_mutex_};
    std::swap(pending_commands_, applying_commands_);
  }
  for (const auto& [command, done] : applying_commands_) {
    const CommandResult result = apply(command);
    if (done) {
      done(result);
    }
  }
  applying_commands_.clear();
}

CommandResult SimRunner::apply(const Command& command) {
  return std::visit(
      Overloaded{
          [&](const SetPilotInput& c) -> CommandResult {
            if (c.vehicle.empty()) {
              for (Vehicle& vehicle : vehicles_) {
                vehicle.controller->set_pilot_input(c.input);
              }
              return {};
            }
            const auto index = find_vehicle(c.vehicle);
            if (!index) {
              return {Status::not_found, std::format("there is no vehicle named '{}'", c.vehicle)};
            }
            vehicles_[*index].controller->set_pilot_input(c.input);
            return {};
          },
          [&](const Reset&) -> CommandResult {
            simulation_.reset();
            for (Vehicle& vehicle : vehicles_) {
              vehicle.controller->reset();
              vehicle.answer.reset();  // Meant for the state before the reset.
            }
            return {};
          },
          [&](const SetPaused& c) -> CommandResult {
            paused_ = c.paused;
            return {};
          },
          [&](const SetSpeed& c) -> CommandResult {
            auto* real_time = std::get_if<RealTime>(&clock_);
            if (real_time == nullptr) {
              return {Status::unsupported, "this simulator runs as fast as it can, not paced to the clock"};
            }
            if (!(c.speed > 0.0)) {
              return {Status::invalid, "speed must be positive"};
            }
            real_time->speed = c.speed;
            repace_ = true;
            return {};
          },
          [&](const SpawnVehicle& c) -> CommandResult {
            if (find_vehicle(c.vehicle.name)) {
              return {Status::exists, std::format("there already is a vehicle named '{}'", c.vehicle.name)};
            }
            try {
              add(c.vehicle);
            } catch (const std::invalid_argument& error) {
              return {Status::invalid, error.what()};
            } catch (const std::exception& error) {
              return {Status::failed, error.what()};
            }
            return {};
          },
          [&](const RemoveVehicle& c) -> CommandResult {
            const auto index = find_vehicle(c.name);
            if (!index) {
              return {Status::not_found, std::format("there is no vehicle named '{}'", c.name)};
            }
            remove(*index);
            return {};
          },
          [&](const SetObstacle& c) -> CommandResult { return set_obstacle(c.obstacle); },
          [&](const RemoveObstacle& c) -> CommandResult {
            auto obstacles = std::make_shared<Obstacles>(*obstacles_);
            if (std::erase_if(obstacles->obstacles, [&](const Obstacle& o) { return o.name == c.name; }) == 0) {
              return {Status::not_found, std::format("there is no obstacle named '{}'", c.name)};
            }
            ++obstacles->version;
            obstacles_ = std::move(obstacles);
            return {};
          },
      },
      command);
}

CommandResult SimRunner::set_obstacle(const Obstacle& obstacle) {
  if (!is_valid_name(obstacle.name)) {
    return {Status::invalid, std::format("invalid obstacle name '{}'; use up to {} letters, digits, '-', '_' and '.', "
                                         "not starting with '.'",
                                         obstacle.name, max_name_length)};
  }
  const Vec3& size = obstacle.size;
  const bool positive = size.x > 0.0 && size.z > 0.0 && (obstacle.shape == ObstacleShape::cylinder || size.y > 0.0);
  if (!is_finite(obstacle.position) || !is_finite(size) || !std::isfinite(obstacle.yaw) || !positive) {
    return {Status::invalid, "an obstacle's position, size and yaw must be finite, and its size positive"};
  }

  auto obstacles = std::make_shared<Obstacles>(*obstacles_);
  const auto existing = std::ranges::find(obstacles->obstacles, obstacle.name, &Obstacle::name);
  if (existing != obstacles->obstacles.end()) {
    *existing = obstacle;
  } else if (obstacles->obstacles.size() >= max_obstacles) {
    return {Status::invalid, std::format("there can be at most {} obstacles", max_obstacles)};
  } else {
    obstacles->obstacles.push_back(obstacle);
  }
  ++obstacles->version;
  obstacles_ = std::move(obstacles);
  return {};
}

std::optional<std::size_t> SimRunner::find_vehicle(const std::string& name) const {
  const auto& vehicles = roster_->vehicles;
  const auto found = std::ranges::find(vehicles, name, &VehicleSpec::name);
  if (found == vehicles.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(found - vehicles.begin());
}

bool SimRunner::step() {
  // Every controller that hasn't answered is asked, even after one wasn't ready, so
  // that they all work on the step at the same time.
  bool all_answered = true;
  for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    Vehicle& vehicle = vehicles_[i];
    if (vehicle.answer) {
      continue;
    }
    const sim::Vehicle& simulated = simulation_.vehicles()[i];
    vehicle.answer = vehicle.controller->update({
        .step = simulation_.step_count(),
        .time = simulation_.time(),
        .step_size = simulation_.step_size(),
        .sensors = simulated.sensors(),
        .truth = simulated.state(),
    });
    all_answered = all_answered && vehicle.answer.has_value();
  }
  if (!all_answered) {
    return false;
  }

  for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    simulation_.set_motor_commands(i, *vehicles_[i].answer);
    vehicles_[i].answer.reset();
  }
  simulation_.step();
  return true;
}

void SimRunner::publish() {
  next_snapshot_.time = simulation_.time();
  next_snapshot_.roster = roster_;
  next_snapshot_.obstacles = obstacles_;
  next_snapshot_.real_time_factor = real_time_factor_;
  next_snapshot_.paused = paused_;
  for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    const sim::Vehicle& vehicle = simulation_.vehicles()[i];
    next_snapshot_.vehicles[i] = {
        .state = vehicle.state(),
        .sensors = vehicle.sensors(),
        .waiting_for_controller = waiting_for_controllers_ && !vehicles_[i].answer,
    };
  }
  // Copy-assigned into a slot whose vector already has the capacity.
  snapshots_.publish(next_snapshot_);
  for (const auto& reader : other_readers_) {
    reader->publish(next_snapshot_);
  }
}

}  // namespace sim::runtime
