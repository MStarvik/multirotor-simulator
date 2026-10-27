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

  // Simulator -> client.
  SIM_CTL_REPLY = 48,  // struct sim_ctl_reply
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

static_assert(sizeof(struct sim_header) == 16, "unexpected padding in sim_header");
static_assert(sizeof(struct sim_viz_marker) == 48, "unexpected padding in sim_viz_marker");
static_assert(sizeof(struct sim_viz_path) == 40, "unexpected padding in sim_viz_path");
static_assert(sizeof(struct sim_ctl_pause) == 24, "unexpected padding in sim_ctl_pause");
static_assert(sizeof(struct sim_ctl_set_speed) == 24, "unexpected padding in sim_ctl_set_speed");
static_assert(sizeof(struct sim_ctl_pilot_input) == 80, "unexpected padding in sim_ctl_pilot_input");
static_assert(sizeof(struct sim_ctl_reply) == 136, "unexpected padding in sim_ctl_reply");
