# Visualization protocol

Defined in [`protocol/multirotor_sim/viz.h`](../protocol/multirotor_sim/viz.h). Any
process, including the flight controller, can draw in the viewer: position
setpoints, planned trajectories, flown paths and so on.

Frames and units follow the simulator's [conventions](../README.md#conventions):
SI units, world frame NED, body frame FRD.

**Transport.** A UNIX domain socket of type `SOCK_SEQPACKET` at
`<socket dir>/viz.sock`. Any number of clients may connect. Messages only go to
the simulator; nothing is sent back. The socket only exists while a viewer or the
recorder runs, so clients should treat it as optional.

**Connect whenever it's there.** A viewer may start after a client, or restart while
it runs. Rather than connecting once at startup, retry now and then (say once a
second) while not connected, and drop the connection when a send fails with anything
but `EAGAIN`. A newly started viewer has no drawings, so after connecting, send the
ones that don't change by themselves, such as a planned path. The simulator creates
this socket before the SITL socket, so a flight controller always finds it once the
SITL socket has accepted it.

**Never block on it.** Send with `MSG_DONTWAIT` and drop messages that would block,
so that drawing can never stall a flight controller, and through it the simulation.
The viewer reads the socket on its own thread, so this only matters when the viewer
falls far behind.

**Drawings** are identified by a 32-bit id chosen by the client, shared by all
clients. Sending a drawing with an id that is in use replaces it, so moving a
setpoint means sending it again. Drawings stay until removed or cleared, also after
their client disconnects. Positions are in the world frame (NED, so height is −z),
in meters; colors are linear RGB in [0, 1].

Every message starts with
`struct sim_viz_header { uint32_t type; uint32_t version; uint32_t id; uint32_t reserved; }`,
where `version` is `SIM_VIZ_PROTOCOL_VERSION` (1).

| Type | Message | Effect |
|---|---|---|
| `SIM_VIZ_MARKER` | `sim_viz_marker` (48 bytes): `position[3]`, `color[3]`, `size` (diameter, m) | A point such as a setpoint: a diamond with a line down to the ground |
| `SIM_VIZ_PATH` | `sim_viz_path` (40 bytes): `color[3]`, `width` (m), `point_count`; then `point_count` × 3 floats | A polyline such as a trajectory, at most 4096 points |
| `SIM_VIZ_REMOVE` | header only | Removes the drawing with `header.id` |
| `SIM_VIZ_CLEAR` | header only | Removes all drawings |

The viewer drops malformed messages (wrong size, unknown type or version) with a
message on stderr. Example, drawing a setpoint 2 m above the origin:

```c
struct sim_viz_marker marker = {
    .header = {.type = SIM_VIZ_MARKER, .version = SIM_VIZ_PROTOCOL_VERSION, .id = 1},
    .position = {0.0f, 0.0f, -2.0f},
    .color = {1.0f, 0.8f, 0.1f},
    .size = 0.15f,
};
send(viz_fd, &marker, sizeof marker, MSG_DONTWAIT | MSG_NOSIGNAL);
```
