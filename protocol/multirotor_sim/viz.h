#pragma once

// Wire format for drawing in the simulator's viewer, e.g. position setpoints and
// planned trajectories. C23 and C++ compatible.
//
// Transport: a UNIX domain socket of type SOCK_SEQPACKET at <socket dir>/viz.sock.
// Any number of clients may connect; each send() is one message. Nothing is sent
// back. The socket only exists while a viewer runs, so treat it as optional.
//
// Send with MSG_DONTWAIT and drop messages that would block, so that drawing can
// never stall a flight controller (and through it, the simulation).
//
// Drawings are identified by a client-chosen id. Each connection has its own ids, so
// clients can't clash. Sending a drawing with an id that is in use replaces it, so
// updating a setpoint is just sending it again. Drawings stay until removed, cleared,
// or their connection closes. Positions are in the world frame (NED), in meters.
// Colors are linear RGB in [0, 1].

#include <stddef.h>
#include <stdint.h>

#define SIM_VIZ_PROTOCOL_VERSION 1u
#define SIM_VIZ_MAX_PATH_POINTS 4096u

enum sim_viz_message_type : uint32_t {
  SIM_VIZ_MARKER = 1,  // struct sim_viz_marker
  SIM_VIZ_PATH = 2,    // struct sim_viz_path, followed by its points
  SIM_VIZ_REMOVE = 3,  // struct sim_viz_header; removes this client's drawing with header.id
  SIM_VIZ_CLEAR = 4,   // struct sim_viz_header; removes all of this client's drawings
};

struct sim_viz_header {
  uint32_t type;     // enum sim_viz_message_type
  uint32_t version;  // SIM_VIZ_PROTOCOL_VERSION
  uint32_t id;       // The drawing this message is about. Ignored by SIM_VIZ_CLEAR.
  uint32_t reserved;
};

// A point of interest, such as a position setpoint. Drawn as a diamond with a thin
// line down to the ground, to make its height readable.
struct sim_viz_marker {
  struct sim_viz_header header;
  float position[3];
  float color[3];
  float size;  // Diameter, m.
  float reserved;
};

// A polyline, such as a planned trajectory. The message continues with point_count
// points of three floats each (x, y, z), so its total size is
// sizeof(struct sim_viz_path) + point_count * 3 * sizeof(float).
struct sim_viz_path {
  struct sim_viz_header header;
  float color[3];
  float width;           // Line width, m.
  uint32_t point_count;  // At most SIM_VIZ_MAX_PATH_POINTS.
  uint32_t reserved;
};

static_assert(sizeof(struct sim_viz_header) == 16, "unexpected padding in sim_viz_header");
static_assert(sizeof(struct sim_viz_marker) == 48, "unexpected padding in sim_viz_marker");
static_assert(sizeof(struct sim_viz_path) == 40, "unexpected padding in sim_viz_path");
