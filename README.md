# Multirotor simulator

A small multirotor simulator for Linux, built for testing a flight control
stack in software-in-the-loop (SITL). It simulates the rigid-body dynamics of a
multirotor and its rotors, an IMU and a barometer, for one vehicle or several. A
flight controller per vehicle connects over a UNIX domain socket and runs in
lockstep with the simulation. Viewers written directly against Vulkan and Wayland
show the vehicles, and other processes can draw setpoints and trajectories in them
and control the simulation.

The simulation never depends on a viewer. Viewers are separate processes that
attach to a running simulator over its socket, any number at a time, each with its
own camera. The simulator itself needs no window or GPU, and can run unpaced by the
wall clock, as fast as the flight controllers allow.

![The example SITL flight controller flying a circle](docs/media/circle.avif)

*The example SITL flight controller flying a circle, drawing its position setpoint
(yellow), planned path (magenta) and flown trail (cyan) over the simulator
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
# Simulator with a viewer, flown from the keyboard by the pilot
./build/release/multirotor-sim --viewer &
./build/release/multirotor-sim-pilot

# Simulator alone, and a viewer attached later, chasing the vehicle
./build/release/multirotor-sim &
./build/release/multirotor-sim-viewer --camera chase

# The demo flight, unpaced, printing the state every simulated second
./build/release/multirotor-sim --unpaced --duration 10 &
./build/release/multirotor-sim-pilot --demo

# Record a video of the demo flight, rendered offscreen
./build/release/multirotor-sim-record --duration 10 --output flight.webm &
./build/release/multirotor-sim-pilot --demo

# Three vehicles, each waiting for its own flight controller
./build/release/multirotor-sim --vehicle alpha --vehicle bravo --vehicle charlie

# Control a running simulator: add or remove vehicles and obstacles, pause, change speed
./build/release/multirotor-sim-ctl spawn delta --at 0,6 --rotors 6
./build/release/multirotor-sim-ctl remove delta
./build/release/multirotor-sim-ctl obstacle wall box --at 5,0 --size 0.3,6,3
./build/release/multirotor-sim-ctl obstacle pole cylinder --at 3,3 --size 0.4,0,4
./build/release/multirotor-sim-ctl pause
```

All programs take the same options; `--help` lists them. The main ones:

| Option | Meaning |
|---|---|
| `--vehicle NAME`, `--vehicles N` | Add a named vehicle (repeatable), or N vehicles named 0 to N−1 (default: one vehicle, `0`) |
| `--rotors N`, `--arm-length M`, `--mass KG` | The vehicles, generated as a symmetric frame (default: 1 kg quadrotor) |
| `--instance NAME` | Names this simulator, so that several can run at once (default: `default`) |
| `--socket-dir DIR` | Where the sockets are created (default: `$XDG_RUNTIME_DIR/multirotor-sim/<instance>`) |
| `--seed N` | Sensor noise seed; equal seeds and inputs give identical runs |
| `--speed F` | Simulator: speed relative to real time |
| `--unpaced` | Simulator: run as fast as the flight controllers allow |
| `--duration S` | Simulator: exit after S simulated seconds; recorder: the length of the video |
| `--viewer` | Simulator: open a viewer |
| `--demo` | Pilot: fly the demo flight instead of the sticks |
| `--msaa N` | Viewer and recorder: anti-aliasing samples, 1 (off), 2, 4 or 8 |
| `--follow NAME` | Viewer and recorder: the vehicle the camera follows |
| `--camera MODE` | Viewer and recorder: `follow` (from the south-west, above), `chase` (from behind), `overhead`, or `fixed` (from `--camera-position N,E,D`) |
| `--camera-distance M` | Viewer and recorder: distance from the camera to the vehicle |
| `--output FILE`, `--size WxH`, `--fps N` | Recorder: the output file, its size and frame rate |

Viewer keys: **W/S** climb and descend, **arrow keys** pitch and roll, **A/D**
yaw, **Tab** follow the next vehicle, **C** change camera mode, **Space** pause,
**R** reset, **Esc** quit. The keys fly the vehicle the camera follows, as stick
inputs forwarded to its flight controller. The window title shows simulation time, altitude, real-time
factor and GPU time per frame.

The simulator has no flight controller of its own: every vehicle waits for one on
its SITL socket, and the simulation steps once all of them have answered.
`multirotor-sim-pilot` is a simple one, flying each vehicle named by `--vehicle`
with an attitude controller, from the sticks or with `--demo` a scripted flight.
It reads the true state rather than estimating it from the sensors. Started
before the simulator, it waits for it, and it exits when the simulator does.

`multirotor-sim` runs until interrupted, for `--duration` simulated seconds, or
with `--viewer` until the viewer is closed. `multirotor-sim-viewer` attaches to the
simulator named by `--instance`; started before it, or after it exits, the viewer
waits for one.

### Several vehicles and simulators

Vehicles start on the ground in a row heading east from the origin, a couple of
meters apart, and more can be added and removed while the simulator runs, with
`multirotor-sim-ctl` or any client of the simulator socket. They don't collide with
each other, nor yet with obstacles, which are only shown for now. Each waits for
its own flight controller, so the simulation doesn't start until every one has
connected, and with `--unpaced`, a simulator without vehicles waits for one.

Each simulator keeps its sockets in a directory of its own, named by `--instance`:

```
$XDG_RUNTIME_DIR/multirotor-sim/
├── default/
│   ├── sim.sock          drawings and control, for the whole simulation
│   └── vehicles/
│       └── 0.sock        SITL, one per vehicle
└── swarm-test/           a second simulator, started with --instance swarm-test
    ├── sim.sock
    └── vehicles/
        ├── alpha.sock
        └── bravo.sock
```

A simulator claims its directory while it runs and removes it when it exits, so
listing `multirotor-sim/` shows what is running, and a second simulator with the
same instance name fails to start; give each its own `--instance`.

`examples/sitl_client.c` is a complete SITL flight controller in C. It flies a
circle from where its vehicle starts and draws its setpoint, plan and trail. It
takes `--instance`, `--socket-dir` and `--vehicle` like the simulator:

```sh
./build/release/multirotor-sim &
./build/release/sitl-example-client

./build/release/multirotor-sim --instance pair --vehicles 2 &
./build/release/sitl-example-client --instance pair --vehicle 0 &
./build/release/sitl-example-client --instance pair --vehicle 1
```

### Recording video

`multirotor-sim-record` runs the simulation unpaced like `multirotor-sim
--unpaced`, renders the same scene as the viewer offscreen, and encodes it with
`ffmpeg`. Frames are taken at fixed steps of simulated time, so the video plays at
real speed however long it took to render. Like the simulator, it waits for a
flight controller per vehicle, and it shows drawings from the simulator socket.

The output's extension picks the format: `.webm` for a VP9 video, or `.avif` for an
animated AV1 image, which browsers play and loop on their own wherever images are
shown, such as in this README. The animation above was made with:

```sh
./build/release/sitl-example-client &
./build/release/multirotor-sim-record --duration 17 --msaa 8 --camera-distance 7 \
    --output docs/media/circle.avif
```

## Structure

| Directory | Contents |
|---|---|
| `src/sim` | Vehicle model, dynamics and sensors. No clocks, threads or graphics. |
| `src/runtime` | Runs the simulation against a clock (real time, or unpaced) and publishes snapshots |
| `src/control` | The attitude controller that `multirotor-sim-pilot` flies with |
| `src/ipc` | The SITL and simulator sockets, and the directory that holds them |
| `src/app` | Sets up the vehicles and their flight controllers from the options, for all programs |
| `src/scene` | What is drawn for a snapshot, shared by the viewer and the recorder |
| `src/platform`, `src/render` | Wayland window; Vulkan rendering to a window or offscreen |
| `src/simulator`, `src/viewer`, `src/pilot`, `src/recorder`, `src/ctl` | The programs |
| `protocol/multirotor_sim` | C headers defining both socket protocols, for clients to include |
| `examples`, `tests` | Example SITL client; tests run by `ctest` |

## Conventions

SI units throughout. The world frame is NED (x north, y east, z down, origin on
the ground); the body frame is FRD (x forward, y right, z down), as in PX4 and
MAVLink. Quaternions are (w, x, y, z) and rotate body-frame vectors into the world
frame. The simulation steps at 1 kHz.

## Protocols

All sockets are UNIX domain sockets of type `SOCK_SEQPACKET` in the socket
directory, with their wire formats defined as C headers in `protocol/multirotor_sim`:

- [SITL protocol](docs/sitl-protocol.md) (`vehicles/<name>.sock`): the lockstep
  exchange of sensor data and motor commands with each vehicle's flight controller.
- [Simulator protocol](docs/sim-protocol.md) (`sim.sock`): markers and paths, such
  as setpoints and planned trajectories, drawn in the viewer by any process; and
  requests that control the simulation, such as pausing it.
