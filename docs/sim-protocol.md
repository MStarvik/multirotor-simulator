# Simulator protocol

Defined in [`protocol/multirotor_sim/sim.h`](../protocol/multirotor_sim/sim.h). Any
process, including a flight controller, can use the simulator socket to draw in the
viewers (position setpoints, planned trajectories, flown paths and so on) and to
control the simulation (pause, reset, speed, stick inputs, adding and removing
vehicles and obstacles). Viewers use it to follow the simulation. `multirotor-sim-ctl` sends
requests from the command line.

Frames and units follow the simulator's [conventions](../README.md#conventions):
SI units, world frame NED, body frame FRD.

**Transport.** A UNIX domain socket of type `SOCK_SEQPACKET` at
`<socket dir>/sim.sock`, one per simulator. It exists for as long as the simulator
runs, and the simulator creates it before the SITL sockets, so a flight controller
always finds it once its SITL socket has accepted it. Any number of clients may
connect, and any client may send any message.

Every message starts with
`struct sim_header { uint32_t type; uint32_t version; uint32_t id; uint32_t reserved; }`,
where `version` is `SIM_PROTOCOL_VERSION` (1). Malformed messages are dropped with a
message on the simulator's stderr, or, for requests that ask for a reply, refused
with `SIM_INVALID`.

## Drawings

Drawings are never answered. Send them with `MSG_DONTWAIT` and drop messages that
would block, so that drawing can never stall a flight controller, and through it the
simulation. The simulator reads the socket on its own thread, so this only matters
when it falls far behind.

Drawings are identified by `header.id`, chosen by the client. Each connection has
its own ids, so flight controllers of different vehicles can use the same ones
without clashing. Sending a drawing with an id that is in use replaces it, so moving
a setpoint means sending it again. Drawings stay until removed, cleared, or until
the connection that sent them closes, so a client must stay connected for as long
as its drawings should show. Positions are in the world frame (NED, so height is
−z), in meters; colors are linear RGB in [0, 1].

| Type | Message | Effect |
|---|---|---|
| `SIM_VIZ_MARKER` | `sim_viz_marker` (48 bytes): `position[3]`, `color[3]`, `size` (diameter, m) | A point such as a setpoint: a diamond with a line down to the ground |
| `SIM_VIZ_PATH` | `sim_viz_path` (40 bytes): `color[3]`, `width` (m), `point_count`; then `point_count` × 3 floats | A polyline such as a trajectory, at most 4096 points |
| `SIM_VIZ_REMOVE` | header only | Removes this client's drawing with `header.id` |
| `SIM_VIZ_CLEAR` | header only | Removes all of this client's drawings |

Example, drawing a setpoint 2 m above the origin:

```c
struct sim_viz_marker marker = {
    .header = {.type = SIM_VIZ_MARKER, .version = SIM_PROTOCOL_VERSION, .id = 1},
    .position = {0.0f, 0.0f, -2.0f},
    .color = {1.0f, 0.8f, 0.1f},
    .size = 0.15f,
};
send(fd, &marker, sizeof marker, MSG_DONTWAIT | MSG_NOSIGNAL);
```

## Requests

Requests change the simulation. Each is answered with a `sim_ctl_reply` (136 bytes)
whose `header.id` is the request's, unless the request's `header.id` is 0, which
asks for no reply. Match replies to requests by id: a request that is refused
straight away can be answered before an earlier one that is still being applied. A
client that asks for replies must read them, or they are dropped once its receive
buffer is full.

The reply's `status` is `SIM_OK`, or why the request failed: `SIM_INVALID`
(malformed, or invalid values), `SIM_NOT_FOUND` (no such vehicle), `SIM_EXISTS`
(a vehicle with that name exists), `SIM_UNSUPPORTED` (this simulator can't do that)
or `SIM_FAILED`. Its `message` explains a failure.

Vehicles are named by a `char[32]` holding the vehicle's name, padded with NUL
bytes.

| Type | Message | Effect |
|---|---|---|
| `SIM_CTL_PAUSE` | `sim_ctl_pause` (24 bytes): `paused` (1 or 0) | Pauses or resumes the simulation |
| `SIM_CTL_RESET` | header only | Puts every vehicle back where it started, and simulation time back to 0 |
| `SIM_CTL_SET_SPEED` | `sim_ctl_set_speed` (24 bytes): `speed` | Speed relative to real time. `SIM_UNSUPPORTED` for a simulator that isn't paced to the clock, such as a headless one without `--realtime`. |
| `SIM_CTL_PILOT_INPUT` | `sim_ctl_pilot_input` (80 bytes): `vehicle`, `sticks[4]` | Roll, pitch, yaw and throttle sticks in [−1, 1] for a vehicle's flight controller, until replaced. The built-in controller flies with them; an external one gets them in `sim_sitl_sensors.pilot`. |
| `SIM_CTL_SPAWN_VEHICLE` | `sim_ctl_spawn_vehicle` (104 bytes): `name`, `controller`, `rotor_count`, `arm_length`, `mass`, `position[3]`, `yaw` | Adds a vehicle, which stays until removed (also after the client disconnects). See below. |
| `SIM_CTL_REMOVE_VEHICLE` | `sim_ctl_remove_vehicle` (48 bytes): `name` | Removes a vehicle. If it had an external flight controller, that sees its SITL socket close. |
| `SIM_CTL_SET_OBSTACLE` | `sim_ctl_set_obstacle` (128 bytes): a `sim_obstacle` | Adds an obstacle, or replaces the one with the same name, which is how to move one. See below. |
| `SIM_CTL_REMOVE_OBSTACLE` | `sim_ctl_remove_obstacle` (48 bytes): `name` | Removes an obstacle |

**Spawning.** A spawned vehicle is a symmetric frame like the simulator's own:
`rotor_count` (even, 4 to 12), `arm_length` (m) and `mass` (kg) left at 0 take the
simulator's values from its command line. It starts at `position` (world frame;
`z` ≤ 0, since down is positive) with heading `yaw` (radians clockwise from north),
and a reset puts it back there. Its name must be unique and valid as a file name:
up to 32 letters, digits, `-`, `_` and `.`, not starting with `.`.

`controller` is `SIM_CONTROLLER_BUILT_IN` for the simulator's own attitude
controller, flown with `SIM_CTL_PILOT_INPUT`, or `SIM_CONTROLLER_SITL` for an
external flight controller on `<socket dir>/vehicles/<name>.sock`. That socket
exists once the reply is sent. Like every SITL vehicle, it holds the whole
simulation until its flight controller connects.

**Obstacles** are boxes and upright cylinders that stay until removed (also after
the client that added them disconnects). For now they are only shown; vehicles fly
through them. A `sim_obstacle` (112 bytes) has a `name`, unique among obstacles and
valid like a vehicle's; a `shape` (`SIM_OBSTACLE_BOX` or `SIM_OBSTACLE_CYLINDER`);
the `position[3]` of the center of its base; its `size[3]`, which for a box is its
length along its heading, width and height, and for a cylinder its diameter, an
unused value and its height; a `yaw` (radians clockwise from north); and a linear
RGB `color[3]`. There can be at most 1024.

Example, pausing and checking the answer:

```c
struct sim_ctl_pause pause = {
    .header = {.type = SIM_CTL_PAUSE, .version = SIM_PROTOCOL_VERSION, .id = 1},
    .paused = 1,
};
send(fd, &pause, sizeof pause, MSG_NOSIGNAL);

struct sim_ctl_reply reply;
recv(fd, &reply, sizeof reply, 0);
if (reply.status != SIM_OK) {
  fprintf(stderr, "could not pause: %s\n", reply.message);
}
```

## Viewing

A viewer sends `SIM_CTL_SUBSCRIBE` (header only) once, and from then on is sent what
it needs to show the simulation:

- **`SIM_VIEW_VEHICLES`**: `sim_view_vehicles` (32 bytes: `roster_version`,
  `count`), followed by `count` `sim_view_vehicle` (432 bytes each: `name`,
  `controller`, `rotor_count`, `propeller_radius`, and each rotor's `position[3]`
  and `spin`). All vehicles, right after subscribing and whenever one comes or goes;
  `roster_version` increases with every change.
- **`SIM_VIEW_OBSTACLES`**: `sim_view_obstacles` (32 bytes: `version`, `count`),
  followed by `count` `sim_obstacle`. All obstacles, right after subscribing and
  whenever one comes, goes or changes.
- **Drawings**: everyone's, as `SIM_VIZ_MARKER`, `SIM_VIZ_PATH` and `SIM_VIZ_REMOVE`
  messages like the ones clients send, but with `header.id` chosen by the simulator,
  unique among all clients' drawings. Right after subscribing, every drawing there
  is; then whatever changes.

The simulator never waits for a subscriber. It sends the latest vehicles, obstacles
and drawings when the subscriber's socket has room, so a slow subscriber skips changes
but catches up as soon as it reads again.

Any client can ask for the state of the vehicles with `SIM_CTL_GET_STATE` (header
only), which is always answered with `SIM_VIEW_STATE`, carrying the request's
`header.id`: `sim_view_state` (48 bytes: `roster_version`, `time`,
`real_time_factor`, `flags`, `count`), followed by `count`
`sim_view_vehicle_state` (64 bytes each: `position[3]`, `attitude[4]`, `flags`).
The vehicles are in the order of the `SIM_VIEW_VEHICLES` with the same
`roster_version`, which a subscriber always gets first. `flags` has
`SIM_VIEW_PAUSED` for the simulation, and `SIM_VIEW_WAITING` for a vehicle whose
flight controller hasn't answered the current step. `multirotor-sim-viewer` asks
once per frame and shows the latest answer.
