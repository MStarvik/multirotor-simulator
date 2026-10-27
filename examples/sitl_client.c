// Example SITL flight controller for the multirotor simulator, in C23.
//
// Connects to a vehicle's SITL socket, flies a circle from where the vehicle
// starts, and draws the position setpoint, the planned circle and the flown trail
// on the viz socket. When the simulation ends, it prints the RMS position error and
// fails if it is large, so it doubles as an end-to-end test.
//
// For brevity it controls position from the ground truth that the simulator sends
// along. A real flight controller would estimate its state from the IMU and
// barometer instead; the attitude loop here does use the gyroscope.
//
// Usage: sitl-example-client [--instance NAME | --socket-dir DIR] [--vehicle NAME]
// The defaults match the simulator's: instance "default", vehicle "0".

#define _POSIX_C_SOURCE 200809L  // POSIX declarations, which strict C23 mode hides.

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <multirotor_sim/sitl.h>
#include <multirotor_sim/viz.h>

constexpr double gravity = 9.80665;  // m/s²
constexpr double pi = 3.14159265358979323846;

// The flight: climb to hover_altitude above the start point, then fly circles
// through it.
constexpr double hover_altitude = 2.0;  // m
constexpr double circle_start = 4.0;    // s
constexpr double circle_radius = 3.0;   // m
constexpr double circle_period = 12.0;  // s
constexpr double settle_time = 3.0;     // s after circle_start before the error counts
constexpr double max_rms_error = 0.3;   // m

// Controller gains.
constexpr double position_gain = 2.5;          // 1/s²
constexpr double velocity_gain = 3.0;          // 1/s
constexpr double max_horizontal_accel = 5.0;   // m/s²
constexpr double attitude_gain = 100.0;        // 1/s²
constexpr double rate_gain = 20.0;             // 1/s

constexpr unsigned trail_capacity = 600;

typedef struct vec3 {
  double x, y, z;
} vec3;

static vec3 add(vec3 a, vec3 b) { return (vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static vec3 sub(vec3 a, vec3 b) { return (vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static vec3 scale(double s, vec3 v) { return (vec3){s * v.x, s * v.y, s * v.z}; }
static double dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static vec3 cross(vec3 a, vec3 b) {
  return (vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static double norm(vec3 v) { return sqrt(dot(v, v)); }
static vec3 normalize(vec3 v) { return scale(1.0 / norm(v), v); }
static vec3 from_array(const double v[3]) { return (vec3){v[0], v[1], v[2]}; }

// Rotates v by the unit quaternion q = (w, x, y, z).
static vec3 rotate(const double q[4], vec3 v) {
  const vec3 axis = {q[1], q[2], q[3]};
  const vec3 t = scale(2.0, cross(axis, v));
  return add(add(v, scale(q[0], t)), cross(axis, t));
}

typedef struct setpoint {
  vec3 position, velocity, acceleration;  // World frame (NED).
} setpoint;

static setpoint setpoint_at(vec3 start, double time) {
  const vec3 hover = {start.x, start.y, -hover_altitude};
  if (time < circle_start) {
    return (setpoint){.position = hover};
  }
  // A circle centered circle_radius south of the start point.
  const double rate = 2.0 * pi / circle_period;
  const double angle = rate * (time - circle_start);
  const double r = circle_radius;
  return (setpoint){
      .position = add(hover, (vec3){-r + r * cos(angle), r * sin(angle), 0.0}),
      .velocity = {-r * rate * sin(angle), r * rate * cos(angle), 0.0},
      .acceleration = {-r * rate * rate * cos(angle), -r * rate * rate * sin(angle), 0.0},
  };
}

// Position control with a geometric attitude controller (Lee, Leok and McClamroch,
// "Geometric tracking control of a quadrotor UAV on SE(3)", 2010), then a mixer
// that is exact for frames with rotors evenly spaced on a circle.
static void control(const struct sim_sitl_hello* vehicle, const struct sim_sitl_sensors* sensors,
                    setpoint target, double motors[]) {
  const struct sim_sitl_ground_truth* truth = &sensors->truth;
  const vec3 position = from_array(truth->position);
  const vec3 velocity = from_array(truth->velocity);

  // Desired acceleration, with the horizontal part limited.
  vec3 accel = add(target.acceleration,
                   add(scale(position_gain, sub(target.position, position)),
                       scale(velocity_gain, sub(target.velocity, velocity))));
  const double horizontal = hypot(accel.x, accel.y);
  if (horizontal > max_horizontal_accel) {
    accel.x *= max_horizontal_accel / horizontal;
    accel.y *= max_horizontal_accel / horizontal;
  }

  // The rotors must supply m (a - g), with g = (0, 0, g) in NED. They push along
  // body -z, so the desired body z axis points opposite that force. Yaw: north.
  const vec3 force = scale(vehicle->mass, sub(accel, (vec3){0.0, 0.0, gravity}));
  const vec3 z_d = normalize(scale(-1.0, force));
  const vec3 y_d = normalize(cross(z_d, (vec3){1.0, 0.0, 0.0}));
  const vec3 x_d = cross(y_d, z_d);

  const vec3 x_b = rotate(truth->attitude, (vec3){1.0, 0.0, 0.0});
  const vec3 y_b = rotate(truth->attitude, (vec3){0.0, 1.0, 0.0});
  const vec3 z_b = rotate(truth->attitude, (vec3){0.0, 0.0, 1.0});
  const double thrust = -dot(force, z_b);

  // Attitude error e_R = ½ vee(R_dᵀ R − Rᵀ R_d), in the body frame.
  const vec3 attitude_error = scale(0.5, (vec3){dot(z_d, y_b) - dot(y_d, z_b), dot(x_d, z_b) - dot(z_d, x_b),
                                                dot(y_d, x_b) - dot(x_d, y_b)});
  const vec3 rate = from_array(sensors->gyroscope);
  const vec3 angular_accel = sub(scale(-attitude_gain, attitude_error), scale(rate_gain, rate));
  const vec3 torque = {vehicle->inertia[0] * angular_accel.x, vehicle->inertia[1] * angular_accel.y,
                       vehicle->inertia[2] * angular_accel.z};

  // Mixer: each rotor's share of thrust and of each torque axis.
  const unsigned count = vehicle->rotor_count;
  double sum_x2 = 0.0;
  double sum_y2 = 0.0;
  for (unsigned i = 0; i < count; ++i) {
    sum_x2 += vehicle->rotors[i].position[0] * vehicle->rotors[i].position[0];
    sum_y2 += vehicle->rotors[i].position[1] * vehicle->rotors[i].position[1];
  }
  const double torque_per_thrust = vehicle->torque_coefficient / vehicle->thrust_coefficient;
  for (unsigned i = 0; i < count; ++i) {
    const struct sim_sitl_rotor* rotor = &vehicle->rotors[i];
    const double rotor_thrust = thrust / count - torque.x * rotor->position[1] / sum_y2 +
                                torque.y * rotor->position[0] / sum_x2 -
                                torque.z * rotor->spin / (count * torque_per_thrust);
    const double speed = sqrt(fmax(rotor_thrust, 0.0) / vehicle->thrust_coefficient);
    motors[i] = fmin(speed / vehicle->max_rotor_speed, 1.0);
  }
}

static int connect_to(const char* path, int attempts) {
  for (int attempt = 1;; ++attempt) {
    const int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    struct sockaddr_un address = {.sun_family = AF_UNIX};
    snprintf(address.sun_path, sizeof address.sun_path, "%s", path);
    if (fd >= 0 && connect(fd, (const struct sockaddr*)&address, sizeof address) == 0) {
      return fd;
    }
    if (fd >= 0) {
      close(fd);
    }
    if (attempt >= attempts) {
      return -1;
    }
    nanosleep(&(struct timespec){.tv_nsec = 100'000'000}, nullptr);
  }
}

// The connection to the viewer's drawing socket. It only exists while a viewer runs,
// which may start after us or restart, so connect whenever it is there.
typedef struct viz_connection {
  const char* path;
  int fd;               // -1 while not connected.
  double next_attempt;  // Monotonic time of the next connection attempt, s.
} viz_connection;

static double monotonic_seconds(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

// Drawing is best effort: never block, and drop messages that would. Any other
// failure means the viewer has gone away.
static void send_drawing(viz_connection* viz, const void* message, size_t size) {
  if (viz->fd >= 0 && send(viz->fd, message, size, MSG_DONTWAIT | MSG_NOSIGNAL) < 0 && errno != EAGAIN) {
    close(viz->fd);
    viz->fd = -1;
  }
}

static void send_marker(viz_connection* viz, uint32_t id, vec3 position, const float color[3], float size) {
  struct sim_viz_marker marker = {
      .header = {.type = SIM_VIZ_MARKER, .version = SIM_VIZ_PROTOCOL_VERSION, .id = id},
      .position = {(float)position.x, (float)position.y, (float)position.z},
      .color = {color[0], color[1], color[2]},
      .size = size,
  };
  send_drawing(viz, &marker, sizeof marker);
}

static void send_path(viz_connection* viz, uint32_t id, const float points[][3], uint32_t count,
                      const float color[3], float width) {
  static unsigned char buffer[sizeof(struct sim_viz_path) + SIM_VIZ_MAX_PATH_POINTS * 3 * sizeof(float)];
  const struct sim_viz_path path = {
      .header = {.type = SIM_VIZ_PATH, .version = SIM_VIZ_PROTOCOL_VERSION, .id = id},
      .color = {color[0], color[1], color[2]},
      .width = width,
      .point_count = count,
  };
  const size_t points_size = (size_t)count * 3 * sizeof(float);
  memcpy(buffer, &path, sizeof path);
  memcpy(buffer + sizeof path, points, points_size);
  send_drawing(viz, buffer, sizeof path + points_size);
}

static void send_planned_circle(viz_connection* viz, vec3 start) {
  constexpr unsigned segments = 96;
  static float points[segments + 1][3];
  for (unsigned i = 0; i <= segments; ++i) {
    const setpoint point = setpoint_at(start, circle_start + circle_period * i / segments);
    points[i][0] = (float)point.position.x;
    points[i][1] = (float)point.position.y;
    points[i][2] = (float)point.position.z;
  }
  send_path(viz, 2, points, segments + 1, (const float[3]){0.85f, 0.15f, 0.75f}, 0.03f);
}

// Connects to the drawing socket if not connected, trying at most once a second. A
// viewer starts without drawings, so the lasting ones are sent on every connect.
// Drawing ids are our own, so they can't clash with another client's.
static void maintain_viz_connection(viz_connection* viz, vec3 start) {
  if (viz->fd >= 0 || monotonic_seconds() < viz->next_attempt) {
    return;
  }
  viz->next_attempt = monotonic_seconds() + 1.0;
  viz->fd = connect_to(viz->path, 1);
  if (viz->fd >= 0) {
    printf("Drawing in the viewer\n");
    send_planned_circle(viz, start);
  }
}

static void usage_error(const char* program) {
  fprintf(stderr, "usage: %s [--instance NAME | --socket-dir DIR] [--vehicle NAME]\n", program);
  exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {
  const char* instance = "default";
  const char* socket_dir = nullptr;
  const char* vehicle_name = "0";
  for (int i = 1; i < argc; ++i) {
    if (i + 1 >= argc) {
      usage_error(argv[0]);
    }
    if (strcmp(argv[i], "--instance") == 0) {
      instance = argv[++i];
    } else if (strcmp(argv[i], "--socket-dir") == 0) {
      socket_dir = argv[++i];
    } else if (strcmp(argv[i], "--vehicle") == 0) {
      vehicle_name = argv[++i];
    } else {
      usage_error(argv[0]);
    }
  }

  // The simulator's socket directory: <runtime dir>/multirotor-sim/<instance>.
  char directory[256];
  if (socket_dir != nullptr) {
    snprintf(directory, sizeof directory, "%s", socket_dir);
  } else if (getenv("XDG_RUNTIME_DIR") != nullptr) {
    snprintf(directory, sizeof directory, "%s/multirotor-sim/%s", getenv("XDG_RUNTIME_DIR"), instance);
  } else {
    snprintf(directory, sizeof directory, "/tmp/multirotor-sim-%u/%s", (unsigned)getuid(), instance);
  }
  char sitl_path[320];
  char viz_path[300];
  snprintf(sitl_path, sizeof sitl_path, "%s/vehicles/%s.sock", directory, vehicle_name);
  snprintf(viz_path, sizeof viz_path, "%s/viz.sock", directory);

  // The simulator may still be starting, so keep trying for a while.
  const int sitl = connect_to(sitl_path, 100);
  if (sitl < 0) {
    fprintf(stderr, "error: could not connect to %s\n", sitl_path);
    return EXIT_FAILURE;
  }
  printf("Connected to %s\n", sitl_path);
  viz_connection viz = {.path = viz_path, .fd = -1, .next_attempt = 0.0};

  struct sim_sitl_hello vehicle;
  if (recv(sitl, &vehicle, sizeof vehicle, 0) != sizeof vehicle || vehicle.header.type != SIM_SITL_HELLO ||
      vehicle.header.version != SIM_SITL_PROTOCOL_VERSION) {
    fprintf(stderr, "error: expected a hello message of protocol version %u\n", SIM_SITL_PROTOCOL_VERSION);
    return EXIT_FAILURE;
  }
  printf("Vehicle: %u rotors, %.2f kg, step %.3f ms\n", vehicle.rotor_count, vehicle.mass,
         (double)vehicle.step_ns * 1e-6);

  static float trail[trail_capacity][3];
  uint32_t trail_length = 0;
  vec3 start = {0};
  bool started = false;
  double squared_error_sum = 0.0;
  unsigned long error_samples = 0;

  // The lockstep loop: one sensor message in, one actuator message out.
  struct sim_sitl_sensors sensors;
  ssize_t length;
  while ((length = recv(sitl, &sensors, sizeof sensors, 0)) > 0) {
    if (length != sizeof sensors || sensors.header.type != SIM_SITL_SENSORS) {
      fprintf(stderr, "error: unexpected message\n");
      return EXIT_FAILURE;
    }
    if (sensors.flags & SIM_SITL_RESET) {
      trail_length = 0;
      squared_error_sum = 0.0;
      error_samples = 0;
      started = false;
    }
    if (!started) {
      start = from_array(sensors.truth.position);
      started = true;
    }

    const double time = (double)sensors.time_ns * 1e-9;
    const setpoint target = setpoint_at(start, time);
    struct sim_sitl_actuators actuators = {
        .header = {.type = SIM_SITL_ACTUATORS, .version = SIM_SITL_PROTOCOL_VERSION},
        .sequence = sensors.sequence,
    };
    control(&vehicle, &sensors, target, actuators.motors);
    if (send(sitl, &actuators, sizeof actuators, MSG_NOSIGNAL) != sizeof actuators) {
      fprintf(stderr, "error: lost the connection to the simulator\n");
      return EXIT_FAILURE;
    }

    const vec3 position = from_array(sensors.truth.position);
    if (time >= circle_start + settle_time) {
      const vec3 error = sub(position, target.position);
      squared_error_sum += dot(error, error);
      ++error_samples;
    }

    // Draw at 50 Hz of simulated time, and extend the trail at 10 Hz.
    maintain_viz_connection(&viz, start);
    const uint64_t step_ms = vehicle.step_ns / 1'000'000 > 0 ? vehicle.step_ns / 1'000'000 : 1;
    if (sensors.sequence % (20 / step_ms) == 0) {
      send_marker(&viz, 1, target.position, (const float[3]){1.0f, 0.8f, 0.1f}, 0.15f);
    }
    if (sensors.sequence % (100 / step_ms) == 0) {
      if (trail_length == trail_capacity) {
        memmove(trail[0], trail[1], (trail_capacity - 1) * sizeof trail[0]);
        --trail_length;
      }
      trail[trail_length][0] = (float)position.x;
      trail[trail_length][1] = (float)position.y;
      trail[trail_length][2] = (float)position.z;
      ++trail_length;
      send_path(&viz, 3, trail, trail_length, (const float[3]){0.1f, 0.9f, 0.9f}, 0.02f);
    }
  }

  // The simulator closed the connection: the run is over.
  if (error_samples == 0) {
    fprintf(stderr, "error: the simulation ended before the circle was flown\n");
    return EXIT_FAILURE;
  }
  const double rms_error = sqrt(squared_error_sum / (double)error_samples);
  printf("RMS position error on the circle: %.3f m\n", rms_error);
  return rms_error <= max_rms_error ? EXIT_SUCCESS : EXIT_FAILURE;
}
