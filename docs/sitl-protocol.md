# SITL protocol

Defined in [`protocol/multirotor_sim/sitl.h`](../protocol/multirotor_sim/sitl.h),
which C and C++ clients include directly.

Frames and units follow the simulator's [conventions](../README.md#conventions):
SI units, world frame NED, body frame FRD.

**Transport.** A UNIX domain socket of type `SOCK_SEQPACKET` per vehicle, at
`<socket dir>/vehicles/<vehicle name>.sock`, where the socket directory is
`$XDG_RUNTIME_DIR/multirotor-sim/<instance>` by default (see
[Several vehicles and simulators](../README.md#several-vehicles-and-simulators)).
Every `send()` is one complete message, so no framing is needed. The simulator
serves one flight controller at a time per vehicle. Messages are plain structs in
native byte order, since both ends share a machine. The headers `static_assert`
their sizes to catch layout differences.

**Lockstep.** The simulator drives the exchange, one round trip per step:

```
simulator                               flight controller
    │ ── sim_sitl_hello ──────────────────────▶ │   once, on connect
    │ ── sim_sitl_sensors (sequence n) ───────▶ │
    │ ◀────────────── sim_sitl_actuators (n) ── │
    │    applies the commands, steps 1 ms        │
    │ ── sim_sitl_sensors (sequence n+1) ─────▶ │
    ⋮                                           ⋮
```

Simulated time advances only when the flight controller answers, and with several
vehicles, only when all of their flight controllers have. A slow, paused or
debugged flight controller therefore pauses the simulation rather than falling
behind it. If a flight controller disconnects, the simulation holds where it is
until one connects again, and the newcomer is sent the hello and then the current
step. Vehicles can also be added and removed while the simulator runs (see the
[simulator protocol](sim-protocol.md)); when a vehicle is removed, its socket
closes, and `recv()` returns 0 as when the simulator exits.

Every message starts with `struct sim_sitl_header { uint32_t type; uint32_t version; }`,
where `version` is `SIM_SITL_PROTOCOL_VERSION` (1).

**`sim_sitl_hello`** (simulator → flight controller, 464 bytes): describes the
simulated vehicle. A flight controller with its own vehicle configuration may
ignore it, but must order its motor outputs as the rotors are listed here.

| Field | Meaning |
|---|---|
| `rotor_count` | Number of rotors (at most 12) |
| `step_ns` | Simulation step, ns |
| `mass`, `inertia[3]` | kg; principal moments about body x, y, z, kg m² |
| `thrust_coefficient`, `torque_coefficient` | k_T, k_Q: thrust = k_T ω², drag torque = k_Q ω² |
| `max_rotor_speed` | ω at motor command 1, rad/s |
| `rotors[i].position[3]`, `rotors[i].spin` | Body-frame position; +1 if clockwise seen from above, −1 if counter-clockwise |

**`sim_sitl_sensors`** (simulator → flight controller, 232 bytes, every step):

| Field | Meaning |
|---|---|
| `flags` | `SIM_SITL_BAROMETER_FRESH`: the barometer was sampled this step (50 Hz). `SIM_SITL_RESET`: the simulation was reset since the previous message. |
| `sequence` | Increases by one per step; echo it in the answer |
| `time_ns` | Simulation time of the measurements |
| `accelerometer[3]` | Specific force, body frame, m/s². About (0, 0, −9.81) at rest. |
| `gyroscope[3]` | Angular velocity, body frame, rad/s |
| `pressure`, `temperature` | Barometer, Pa and °C; repeats the latest sample when not fresh |
| `pilot[4]` | Roll, pitch, yaw, throttle sticks from the viewer, in [−1, 1] |
| `truth` | True position, velocity, attitude and angular velocity, for logging and evaluating estimators, not for control |

The IMU is sampled every step, with white noise and random-walk bias. The barometer
follows the standard atmosphere, with noise and slow drift.

**`sim_sitl_actuators`** (flight controller → simulator, 120 bytes, once per
`sim_sitl_sensors`):

| Field | Meaning |
|---|---|
| `sequence` | The `sequence` of the sensor message being answered |
| `motors[12]` | Normalized rotor speed commands in [0, 1], in hello rotor order; command u requests ω = u · `max_rotor_speed` |

An answer of the wrong size or type, protocol version, or sequence number makes
the simulator drop the connection and wait for a new one.

A minimal flight controller loop:

```c
int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
connect(fd, (struct sockaddr*)&address, sizeof address);  // <socket dir>/vehicles/<name>.sock

struct sim_sitl_hello vehicle;
recv(fd, &vehicle, sizeof vehicle, 0);

struct sim_sitl_sensors sensors;
while (recv(fd, &sensors, sizeof sensors, 0) == sizeof sensors) {
  struct sim_sitl_actuators out = {
      .header = {.type = SIM_SITL_ACTUATORS, .version = SIM_SITL_PROTOCOL_VERSION},
      .sequence = sensors.sequence,
  };
  run_flight_controller(&sensors, out.motors);
  send(fd, &out, sizeof out, MSG_NOSIGNAL);
}
// recv() returning 0 means the simulator ended the run.
```
