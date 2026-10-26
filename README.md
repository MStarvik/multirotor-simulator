# Multirotor simulator

A small multirotor simulator for Linux, built for testing a flight control
stack in software-in-the-loop (SITL). It simulates the rigid-body dynamics of a
multirotor and its rotors, an IMU and a barometer. A flight controller connects
over a UNIX domain socket and runs in lockstep with the simulation. A viewer
written directly against Vulkan and Wayland shows the vehicle, and other processes
can draw setpoints and trajectories in it.

The simulation never depends on the viewer. It runs on its own thread, and the same
simulation also runs headless, without a window or GPU, and without being tied to
the wall clock.

![The example SITL flight controller flying a circle](docs/media/circle.avif)

*The example SITL flight controller flying a circle, drawing its position setpoint
(yellow), planned path (magenta) and flown trail (cyan) over the visualization
socket. Rendered offscreen with `multirotor-sim-record`.*

## Building

Requires Linux with CMake 4.0+, Ninja, a C23/C++23 compiler (GCC 15+ or
Clang 19+), and the development packages for Vulkan 1.4 (headers, loader, `glslc`),
Wayland (`wayland-client`, `wayland-scanner`, `wayland-protocols`) and
`xkbcommon`. The viewer needs a Wayland compositor and a Vulkan 1.4 GPU. Recording
video needs a Vulkan 1.4 GPU and `ffmpeg` with VP9 (libvpx) or AV1 (SVT-AV1), but no
display.

```sh
cmake --preset release            # or: debug, debug-clang
cmake --build --preset release
ctest --preset release
```

Builds go to `build/<preset>/`. Debug builds enable the Vulkan validation layers.

## Running

```sh
# Viewer, flown with the built-in controller and the keyboard
./build/release/multirotor-sim

# Viewer, waiting for an external flight controller
./build/release/multirotor-sim --sitl

# Headless: a scripted flight, or an external flight controller with --sitl
./build/release/multirotor-sim-headless --duration 60
./build/release/multirotor-sim-headless --sitl --duration 600

# Record a video of a flight, rendered offscreen
./build/release/multirotor-sim-record --duration 10 --output flight.webm
```

All programs take the same options; `--help` lists them. The main ones:

| Option | Meaning |
|---|---|
| `--rotors N`, `--arm-length M`, `--mass KG` | The vehicle, generated as a symmetric frame (default: 1 kg quadrotor) |
| `--sitl` | Use an external flight controller instead of the built-in one |
| `--socket-dir DIR` | Where the sockets are created (default: `$XDG_RUNTIME_DIR/multirotor-sim`) |
| `--seed N` | Sensor noise seed; equal seeds and inputs give identical runs |
| `--speed F` | Speed relative to real time (viewer, or headless with `--realtime`) |
| `--duration S` | Headless and recorder: simulated seconds to run (headless runs unpaced, unless `--realtime`) |
| `--msaa N` | Viewer and recorder: anti-aliasing samples, 1 (off), 2, 4 or 8 |
| `--camera-distance M` | Viewer and recorder: distance from the follow camera to the vehicle |
| `--output FILE`, `--size WxH`, `--fps N` | Recorder: the output file, its size and frame rate |

Viewer keys: **W/S** climb and descend, **arrow keys** pitch and roll, **A/D**
yaw, **Space** pause, **R** reset, **Esc** quit. With the built-in controller the
keys fly the vehicle. With `--sitl` they are forwarded to the flight controller as
stick inputs. The window title shows simulation time, altitude, real-time factor and
GPU time per frame.

`examples/sitl_client.c` is a complete SITL flight controller in C. It flies a
circle and draws its setpoint, plan and trail:

```sh
./build/release/multirotor-sim --sitl &
./build/release/sitl-example-client
```

### Recording video

`multirotor-sim-record` runs the simulation like the headless program, renders the
same scene as the viewer offscreen, and encodes it with `ffmpeg`. Frames are taken
at fixed steps of simulated time, so the video plays at real speed however long it
took to render. It flies the scripted flight, or follows an external flight
controller with `--sitl`, and shows drawings from the visualization socket.

The output's extension picks the format: `.webm` for a VP9 video, or `.avif` for an
animated AV1 image, which browsers play and loop on their own wherever images are
shown, such as in this README. The animation above was made with:

```sh
./build/release/sitl-example-client &
./build/release/multirotor-sim-record --sitl --duration 17 --msaa 8 --camera-distance 7 \
    --output docs/media/circle.avif
```

## Structure

| Directory | Contents |
|---|---|
| `src/sim` | Vehicle model, dynamics and sensors. No clocks, threads or graphics. |
| `src/runtime` | Runs the simulation against a clock (real time, or unpaced) and publishes snapshots |
| `src/control` | The built-in flight controller, used without `--sitl` |
| `src/ipc` | The SITL and visualization sockets |
| `src/scene` | What is drawn for a snapshot, shared by the viewer and the recorder |
| `src/platform`, `src/render` | Wayland window; Vulkan rendering to a window or offscreen |
| `src/viewer`, `src/headless`, `src/recorder` | The programs |
| `protocol/multirotor_sim` | C headers defining both socket protocols, for clients to include |
| `examples`, `tests` | Example SITL client; tests run by `ctest` |

## Conventions

SI units throughout. The world frame is NED (x north, y east, z down, origin on
the ground); the body frame is FRD (x forward, y right, z down), as in PX4 and
MAVLink. Quaternions are (w, x, y, z) and rotate body-frame vectors into the world
frame. The simulation steps at 1 kHz.

## Protocols

Both sockets are UNIX domain sockets of type `SOCK_SEQPACKET` in the socket
directory, with their wire formats defined as C headers in `protocol/multirotor_sim`:

- [SITL protocol](docs/sitl-protocol.md) (`sitl.sock`): the lockstep exchange of
  sensor data and motor commands with the flight controller.
- [Visualization protocol](docs/viz-protocol.md) (`viz.sock`): markers and paths,
  such as setpoints and planned trajectories, drawn in the viewer by any process.
