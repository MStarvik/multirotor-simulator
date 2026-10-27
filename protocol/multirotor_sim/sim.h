#pragma once

// Wire format for the simulator socket: drawing in the viewers, and controlling the
// simulation. C23 and C++ compatible.
//
// Transport: a UNIX domain socket of type SOCK_SEQPACKET at <socket dir>/sim.sock,
// which exists for as long as the simulator runs. Any number of clients may connect,
// and any client may send any message; each send() is one message.
//
// Drawings (SIM_VIZ_*) are never answered. Send them with MSG_DONTWAIT and drop
// messages that would block, so that drawing can never stall a flight controller
// (and through it, the simulation). Drawings are identified by a client-chosen id.
// Each connection has its own ids, so clients can't clash. Sending a drawing with an
// id that is in use replaces it, so updating a setpoint is just sending it again.
// Drawings stay until removed, cleared, or their connection closes.
//
// Requests (SIM_CTL_*) are answered with a sim_ctl_reply carrying the request's
// header.id, unless that id is 0. Match replies to requests by id: a request that
// is rejected straight away can be answered before an earlier one. A client that
// sends requests with nonzero ids must read the replies, or they are dropped once
// its receive buffer is full.
//
// Viewers subscribe (SIM_CTL_SUBSCRIBE) to be sent what they need to show the
// simulation: the vehicles (SIM_VIEW_VEHICLES) whenever they change, and everyone's
// drawings (SIM_VIZ_*, with ids chosen by the simulator) as they change. The
// simulator sends the latest of these when the subscriber's socket has room, so a
// slow subscriber skips changes but always catches up. Any client can ask for the
// state of the vehicles (SIM_CTL_GET_STATE), answered with SIM_VIEW_STATE.
//
// Units are SI. Frames: world NED (x north, y east, z down), body FRD (x forward,
// y right, z down). Colors are linear RGB in [0, 1].

#include <stddef.h>
#include <stdint.h>

#define SIM_PROTOCOL_VERSION 1u
#define SIM_VIZ_MAX_PATH_POINTS 4096u
// Names of vehicles, as in the vehicle's SITL socket name: up to this many letters,
// digits, '-', '_' and '.', not starting with '.'. Padded with NUL bytes, so a name
// of exactly this length has no terminating NUL.
#define SIM_NAME_SIZE 32u
#define SIM_MAX_VEHICLES 256u
#define SIM_MAX_ROTORS 12u

enum sim_message_type : uint32_t {
  // Drawings, client -> simulator.
  SIM_VIZ_MARKER = 1,  // struct sim_viz_marker
  SIM_VIZ_PATH = 2,    // struct sim_viz_path, followed by its points
  SIM_VIZ_REMOVE = 3,  // struct sim_header; removes this client's drawing with header.id
  SIM_VIZ_CLEAR = 4,   // struct sim_header; removes all of this client's drawings

  // Requests, client -> simulator.
  SIM_CTL_PAUSE = 16,        // struct sim_ctl_pause
  SIM_CTL_RESET = 17,        // struct sim_header; puts every vehicle back where it started
  SIM_CTL_SET_SPEED = 18,    // struct sim_ctl_set_speed
  SIM_CTL_PILOT_INPUT = 19,  // struct sim_ctl_pilot_input
  SIM_CTL_SPAWN_VEHICLE = 20,   // struct sim_ctl_spawn_vehicle
  SIM_CTL_REMOVE_VEHICLE = 21,  // struct sim_ctl_remove_vehicle
  SIM_CTL_SUBSCRIBE = 22,       // struct sim_header
  SIM_CTL_GET_STATE = 23,       // struct sim_header; answered with SIM_VIEW_STATE

  // Simulator -> client.
  SIM_CTL_REPLY = 48,  // struct sim_ctl_reply

  // Simulator -> client, besides SIM_VIZ_* to subscribers.
  SIM_VIEW_VEHICLES = 64,  // struct sim_view_vehicles, followed by its vehicles
  SIM_VIEW_STATE = 65,     // struct sim_view_state, followed by its vehicles' states
};

struct sim_header {
  uint32_t type;     // enum sim_message_type
  uint32_t version;  // SIM_PROTOCOL_VERSION
  // Drawings: the drawing this message is about (ignored by SIM_VIZ_CLEAR).
  // Requests: chosen by the client and echoed in the reply; 0 for no reply.
  uint32_t id;
  uint32_t reserved;
};

// A point of interest, such as a position setpoint. Drawn as a diamond with a thin
// line down to the ground, to make its height readable.
struct sim_viz_marker {
  struct sim_header header;
  float position[3];
  float color[3];
  float size;  // Diameter, m.
  float reserved;
};

// A polyline, such as a planned trajectory. The message continues with point_count
// points of three floats each (x, y, z), so its total size is
// sizeof(struct sim_viz_path) + point_count * 3 * sizeof(float).
struct sim_viz_path {
  struct sim_header header;
  float color[3];
  float width;           // Line width, m.
  uint32_t point_count;  // At most SIM_VIZ_MAX_PATH_POINTS.
  uint32_t reserved;
};

struct sim_ctl_pause {
  struct sim_header header;
  uint32_t paused;  // 1 to pause, 0 to run.
  uint32_t reserved;
};

// Only for a simulator paced to the wall clock; others answer SIM_UNSUPPORTED.
struct sim_ctl_set_speed {
  struct sim_header header;
  double speed;  // Relative to real time, > 0.
};

// Stick inputs for a vehicle's flight controller: the built-in one flies with them,
// and an external one gets them in sim_sitl_sensors.pilot. They stay until replaced.
struct sim_ctl_pilot_input {
  struct sim_header header;
  char vehicle[SIM_NAME_SIZE];
  double sticks[4];  // Roll (+ right), pitch (+ forward), yaw (+ clockwise), throttle (+ up), in [-1, 1].
};

enum sim_controller : uint32_t {
  SIM_CONTROLLER_BUILT_IN = 0,  // The simulator's own, flown with SIM_CTL_PILOT_INPUT.
  SIM_CONTROLLER_SITL = 1,      // External, on <socket dir>/vehicles/<name>.sock.
};

// Adds a vehicle, generated as a symmetric frame like the simulator's own, which
// stays until removed. The frame fields left at 0 take the simulator's values. A
// vehicle with an external flight controller holds the whole simulation until one
// connects to its SITL socket, which exists once the reply has been sent.
struct sim_ctl_spawn_vehicle {
  struct sim_header header;
  char name[SIM_NAME_SIZE];  // Unique in the simulation.
  uint32_t controller;       // enum sim_controller
  uint32_t rotor_count;      // Even, 4 to 12.
  double arm_length;         // From the center to each rotor, m.
  double mass;               // kg
  double position[3];        // Where it starts, and where a reset puts it. World frame, m; z <= 0.
  double yaw;                // Heading, rad clockwise from north.
};

// Removes a vehicle. An external flight controller sees its SITL socket close.
struct sim_ctl_remove_vehicle {
  struct sim_header header;
  char name[SIM_NAME_SIZE];
};

enum sim_status : int32_t {
  SIM_OK = 0,
  SIM_INVALID = 1,      // The request was malformed or had invalid values.
  SIM_NOT_FOUND = 2,    // No vehicle (or other object) has that name.
  SIM_EXISTS = 3,       // Something with that name already exists.
  SIM_UNSUPPORTED = 4,  // This simulator can't do that.
  SIM_FAILED = 5,       // Valid, but it failed; see the message.
};

struct sim_ctl_reply {
  struct sim_header header;  // header.id is the request's.
  int32_t status;            // enum sim_status
  uint32_t reserved;
  char message[112];  // Explains a failure, NUL-terminated. Empty on success.
};

struct sim_view_rotor {
  double position[3];  // Body frame, m.
  double spin;         // +1 if it turns clockwise seen from above, -1 if counter-clockwise.
};

struct sim_view_vehicle {
  char name[SIM_NAME_SIZE];
  uint32_t controller;  // enum sim_controller
  uint32_t rotor_count;
  double propeller_radius;  // m
  struct sim_view_rotor rotors[SIM_MAX_ROTORS];
};

// Simulator -> subscriber: all vehicles, right after subscribing and whenever they
// come or go. Followed by count sim_view_vehicle, in the order of every
// sim_view_state with the same roster_version.
struct sim_view_vehicles {
  struct sim_header header;  // header.id is 0.
  uint64_t roster_version;   // Increases with every change.
  uint32_t count;            // At most SIM_MAX_VEHICLES.
  uint32_t reserved;
};

enum sim_view_vehicle_flags : uint32_t {
  SIM_VIEW_WAITING = 1u << 0,  // Waiting for its flight controller to answer.
};

struct sim_view_vehicle_state {
  double position[3];  // World frame, m.
  double attitude[4];  // Quaternion w, x, y, z rotating body vectors into the world frame.
  uint32_t flags;      // enum sim_view_vehicle_flags
  uint32_t reserved;
};

enum sim_view_state_flags : uint32_t {
  SIM_VIEW_PAUSED = 1u << 0,
};

// Simulator -> client, answering SIM_CTL_GET_STATE (header.id is the request's).
// Followed by count sim_view_vehicle_state, in the order of the sim_view_vehicles
// with the same roster_version.
struct sim_view_state {
  struct sim_header header;
  uint64_t roster_version;
  double time;              // Simulation time, s.
  double real_time_factor;  // Recent simulated seconds per wall-clock second.
  uint32_t flags;           // enum sim_view_state_flags
  uint32_t count;
};

static_assert(sizeof(struct sim_header) == 16, "unexpected padding in sim_header");
static_assert(sizeof(struct sim_viz_marker) == 48, "unexpected padding in sim_viz_marker");
static_assert(sizeof(struct sim_viz_path) == 40, "unexpected padding in sim_viz_path");
static_assert(sizeof(struct sim_ctl_pause) == 24, "unexpected padding in sim_ctl_pause");
static_assert(sizeof(struct sim_ctl_set_speed) == 24, "unexpected padding in sim_ctl_set_speed");
static_assert(sizeof(struct sim_ctl_pilot_input) == 80, "unexpected padding in sim_ctl_pilot_input");
static_assert(sizeof(struct sim_ctl_spawn_vehicle) == 104, "unexpected padding in sim_ctl_spawn_vehicle");
static_assert(sizeof(struct sim_ctl_remove_vehicle) == 48, "unexpected padding in sim_ctl_remove_vehicle");
static_assert(sizeof(struct sim_ctl_reply) == 136, "unexpected padding in sim_ctl_reply");
static_assert(sizeof(struct sim_view_vehicle) == 432, "unexpected padding in sim_view_vehicle");
static_assert(sizeof(struct sim_view_vehicles) == 32, "unexpected padding in sim_view_vehicles");
static_assert(sizeof(struct sim_view_vehicle_state) == 64, "unexpected padding in sim_view_vehicle_state");
static_assert(sizeof(struct sim_view_state) == 48, "unexpected padding in sim_view_state");
