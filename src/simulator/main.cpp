#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <limits>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "app/setup.hpp"
#include "ipc/sim_server.hpp"
#include "ipc/socket_directory.hpp"
#include "runtime/options.hpp"
#include "runtime/sim_runner.hpp"
#include "sim/math.hpp"

// The simulator. Paced to the wall clock, or with --unpaced as fast as the flight
// controllers allow. Every vehicle waits for an external flight controller on its
// SITL socket. Runs until interrupted, for --duration simulated seconds, or with
// --viewer until the viewer it opens (multirotor-sim-viewer, a separate process) is
// closed. Viewers can attach at any time.

extern char** environ;

namespace {

using namespace sim;

// Starts the viewer next to this executable, watching this simulator.
pid_t start_viewer(const runtime::Options& options, const sigset_t& original_signal_mask) {
  const std::filesystem::path viewer = std::filesystem::read_symlink("/proc/self/exe").parent_path() /
                                       "multirotor-sim-viewer";
  std::vector<std::string> arguments{
      viewer.native(),
      "--socket-dir", options.socket_directory.native(),
      "--exit-with-simulator",
      "--msaa", std::to_string(options.msaa_samples),
      "--camera", std::string{to_string(options.camera)},
      "--camera-distance", std::format("{}", options.camera_distance),
      "--camera-position", std::format("{},{},{}", options.camera_position.x, options.camera_position.y,
                                       options.camera_position.z),
  };
  if (!options.follow.empty()) {
    arguments.insert(arguments.end(), {"--follow", options.follow});
  }
  std::vector<char*> argv;
  for (std::string& argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);

  // The viewer gets the signal mask this process started with, not the one blocking
  // the signals that are waited for here.
  posix_spawnattr_t attributes;
  posix_spawnattr_init(&attributes);
  posix_spawnattr_setsigmask(&attributes, &original_signal_mask);
  posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSIGMASK);
  pid_t pid = -1;
  const int error = ::posix_spawn(&pid, viewer.c_str(), nullptr, &attributes, argv.data(), environ);
  posix_spawnattr_destroy(&attributes);
  if (error != 0) {
    throw std::system_error{error, std::generic_category(), std::format("could not start {}", viewer.native())};
  }
  return pid;
}

int run(const runtime::Options& options, const sigset_t& waited_signals, const sigset_t& original_signal_mask) {
  const runtime::ClockPolicy clock = options.unpaced ? runtime::ClockPolicy{runtime::AsFastAsPossible{}}
                                                     : runtime::ClockPolicy{runtime::RealTime{.speed = options.speed}};
  const ipc::SocketDirectory sockets{options.socket_directory};
  runtime::SimRunner runner = app::make_runner(options, sockets, clock);
  const ipc::SimServer server{sockets.sim_socket(), runner, options.frame};
  // After the simulator socket, so that a flight controller that is already
  // waiting finds it as soon as its SITL socket accepts it.
  for (runtime::VehicleSpec& vehicle : app::initial_vehicles(options)) {
    runner.add_vehicle(std::move(vehicle));
  }

  const auto wall_start = std::chrono::steady_clock::now();
  std::atomic<bool> diverged = false;
  // Declared last, so it is stopped and joined before the server and runner go away.
  const std::jthread simulation_thread{[&runner, &options, &diverged](std::stop_token stop) {
    try {
      runner.run(stop, options.duration.value_or(std::numeric_limits<double>::infinity()));
    } catch (const std::exception& error) {
      std::println(stderr, "fatal: simulation failed: {}", error.what());
      std::abort();
    }
    if (!stop.stop_requested()) {
      const runtime::Snapshot& snapshot = runner.latest();
      diverged = !std::ranges::all_of(snapshot.vehicles, [](const runtime::VehicleSnapshot& vehicle) {
        return is_finite(vehicle.state.position) && is_finite(vehicle.state.attitude);
      });
      ::kill(::getpid(), SIGUSR1);  // Reached the duration.
    }
  }};

  std::optional<pid_t> viewer;
  if (options.viewer) {
    viewer = start_viewer(options, original_signal_mask);
  }

  int exit_status = EXIT_SUCCESS;
  while (true) {
    int signal = 0;
    sigwait(&waited_signals, &signal);
    if (signal == SIGUSR1 && options.duration) {
      const std::chrono::duration<double> wall_time = std::chrono::steady_clock::now() - wall_start;
      std::println("Simulated {:.1f} s in {:.3f} s of wall time ({:.0f}× real time)", *options.duration,
                   wall_time.count(), *options.duration / wall_time.count());
      if (diverged) {
        std::println(stderr, "error: the simulation diverged");
        exit_status = EXIT_FAILURE;
      }
      break;
    }
    if (signal != SIGCHLD) {
      break;  // Interrupted or terminated.
    }
    int status = 0;
    if (viewer && ::waitpid(*viewer, &status, WNOHANG) == *viewer) {
      viewer.reset();
      if (!WIFEXITED(status) || WEXITSTATUS(status) != EXIT_SUCCESS) {
        std::println(stderr, "error: the viewer failed");
        exit_status = EXIT_FAILURE;
      }
      break;  // The window was closed.
    }
  }
  if (viewer) {
    ::kill(*viewer, SIGTERM);
    ::waitpid(*viewer, nullptr, 0);
  }
  return exit_status;
}

}  // namespace

int main(int argc, char** argv) {
  // Blocked in every thread, and waited for by the main thread. Before any thread
  // starts, since threads inherit the mask.
  sigset_t waited_signals;
  sigemptyset(&waited_signals);
  sigaddset(&waited_signals, SIGINT);
  sigaddset(&waited_signals, SIGTERM);
  sigaddset(&waited_signals, SIGCHLD);
  sigaddset(&waited_signals, SIGUSR1);  // The simulation reached --duration.
  sigset_t original_signal_mask;
  pthread_sigmask(SIG_BLOCK, &waited_signals, &original_signal_mask);

  try {
    const auto options = runtime::parse_options({argv, static_cast<std::size_t>(argc)});
    return options ? run(*options, waited_signals, original_signal_mask) : EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::println(stderr, "fatal: {}", error.what());
    return EXIT_FAILURE;
  }
}
