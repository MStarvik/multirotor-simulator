#include "runtime/sim_runner.hpp"

#include <chrono>
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

SimRunner::SimRunner(Simulation simulation, std::vector<std::unique_ptr<FlightController>> controllers,
                     ClockPolicy clock)
    : simulation_{std::move(simulation)},
      controllers_{std::move(controllers)},
      answers_(controllers_.size()),
      clock_{clock} {
  if (controllers_.size() != simulation_.vehicles().size()) {
    throw std::invalid_argument{"need one flight controller per vehicle"};
  }
  next_snapshot_.vehicles.resize(controllers_.size());
  publish();
}

void SimRunner::push(Command command) {
  const std::lock_guard lock{commands_mutex_};
  pending_commands_.push_back(command);
}

void SimRunner::run(std::stop_token stop, double until) {
  const double speed = std::holds_alternative<RealTime>(clock_) ? std::get<RealTime>(clock_).speed : 0.0;

  // Real-time pacing: the simulation should be anchor_sim_time + elapsed * speed.
  auto anchor_wall_time = Clock::now();
  double anchor_sim_time = simulation_.time();
  auto rate_wall_time = anchor_wall_time;
  double rate_sim_time = anchor_sim_time;

  while (!stop.stop_requested() && simulation_.time() < until) {
    apply_commands();

    const bool stepped = !paused_ && step();
    waiting_for_controllers_ = !paused_ && !stepped;
    if (!stepped) {
      // Paused, or a flight controller is not ready (it waits briefly itself).
      // Either way, real-time pacing restarts from here rather than catching up.
      real_time_factor_ = 0.0;
      publish();
      if (paused_) {
        std::this_thread::sleep_for(pause_poll_interval);
      }
      anchor_wall_time = rate_wall_time = Clock::now();
      anchor_sim_time = rate_sim_time = simulation_.time();
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

    if (speed > 0.0) {
      const auto target = anchor_wall_time + std::chrono::duration_cast<Clock::duration>(
                                                 Seconds{(simulation_.time() - anchor_sim_time) / speed});
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
  for (const Command& command : applying_commands_) {
    std::visit(Overloaded{
                   [&](const SetPilotInput& c) { controllers_.at(c.vehicle)->set_pilot_input(c.input); },
                   [&](const Reset&) {
                     simulation_.reset();
                     for (std::size_t i = 0; i < controllers_.size(); ++i) {
                       controllers_[i]->reset();
                       answers_[i].reset();  // Meant for the state before the reset.
                     }
                   },
                   [&](const SetPaused& c) { paused_ = c.paused; },
               },
               command);
  }
  applying_commands_.clear();
}

bool SimRunner::step() {
  // Every controller that hasn't answered is asked, even after one wasn't ready, so
  // that they all work on the step at the same time.
  bool all_answered = true;
  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    if (answers_[i]) {
      continue;
    }
    const Vehicle& vehicle = simulation_.vehicles()[i];
    answers_[i] = controllers_[i]->update({
        .step = simulation_.step_count(),
        .time = simulation_.time(),
        .step_size = simulation_.step_size(),
        .sensors = vehicle.sensors(),
        .truth = vehicle.state(),
    });
    all_answered = all_answered && answers_[i].has_value();
  }
  if (!all_answered) {
    return false;
  }

  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    simulation_.set_motor_commands(i, *answers_[i]);
    answers_[i].reset();
  }
  simulation_.step();
  return true;
}

void SimRunner::publish() {
  next_snapshot_.time = simulation_.time();
  next_snapshot_.real_time_factor = real_time_factor_;
  next_snapshot_.paused = paused_;
  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    const Vehicle& vehicle = simulation_.vehicles()[i];
    next_snapshot_.vehicles[i] = {
        .state = vehicle.state(),
        .sensors = vehicle.sensors(),
        .waiting_for_controller = waiting_for_controllers_ && !answers_[i],
    };
  }
  // Copy-assigned into a slot whose vector already has the capacity.
  snapshots_.publish(next_snapshot_);
}

}  // namespace sim::runtime
