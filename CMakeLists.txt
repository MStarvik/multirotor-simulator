cmake_minimum_required(VERSION 4.0...4.4)

project(MultirotorSim
  VERSION 0.1.0
  DESCRIPTION "A minimal multirotor simulator rendering with Vulkan on Wayland"
  LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Standard modes without GNU extensions. The language level itself is requested per
# target through sim_build_options below.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
# No C++20 modules here, so skip scanning every source for them.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------

# Only the headers are linked: Vulkan-Hpp's RAII context loads libvulkan.so.1 at runtime.
# glslc compiles the shaders at build time.
find_package(Vulkan 1.4 REQUIRED COMPONENTS glslc)

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(wayland-client REQUIRED IMPORTED_TARGET wayland-client>=1.23)
pkg_check_modules(xkbcommon REQUIRED IMPORTED_TARGET xkbcommon)

include(Shaders)
include(WaylandProtocols)

# -----------------------------------------------------------------------------
# Shared build options
# -----------------------------------------------------------------------------

add_library(sim_build_options INTERFACE)
target_compile_features(sim_build_options INTERFACE c_std_23 cxx_std_23)

set(sim_common_warnings -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  -Wnull-dereference -Wimplicit-fallthrough)
set(sim_cxx_warnings -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
target_compile_options(sim_build_options INTERFACE
  "$<$<COMPILE_LANG_AND_ID:C,GNU,Clang>:${sim_common_warnings}>"
  "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:${sim_common_warnings};${sim_cxx_warnings}>")

# Cheap bounds checking in the standard library for debug builds.
target_compile_definitions(sim_build_options INTERFACE
  "$<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>")

# -----------------------------------------------------------------------------
# Simulation core: vehicle model and dynamics. No threads, clocks or graphics.
# -----------------------------------------------------------------------------

add_library(sim_core STATIC
  src/sim/multirotor.cpp
  src/sim/sensors.cpp
  src/sim/simulation.cpp)
target_include_directories(sim_core PUBLIC src)
target_link_libraries(sim_core PRIVATE sim_build_options)

# -----------------------------------------------------------------------------
# Runtime: runs the simulation against a clock and publishes snapshots
# -----------------------------------------------------------------------------

add_library(sim_runtime STATIC
  src/runtime/options.cpp
  src/runtime/sim_runner.cpp)
target_link_libraries(sim_runtime
  PUBLIC sim_core Threads::Threads
  PRIVATE sim_build_options)

# -----------------------------------------------------------------------------
# Control: built-in flight controller, used until a real one is connected
# -----------------------------------------------------------------------------

add_library(sim_control STATIC
  src/control/attitude_controller.cpp
  src/control/demo_flight.cpp
  src/control/mixer.cpp)
target_link_libraries(sim_control
  PUBLIC sim_runtime
  PRIVATE sim_build_options)

# -----------------------------------------------------------------------------
# IPC: the SITL and drawing sockets
# -----------------------------------------------------------------------------

# The wire format, as C headers that flight controllers and other clients include.
add_library(sim_protocol INTERFACE)
target_include_directories(sim_protocol INTERFACE protocol)

add_library(sim_ipc STATIC
  src/ipc/sitl_controller.cpp
  src/ipc/socket_directory.cpp
  src/ipc/unix_socket.cpp
  src/ipc/sim_server.cpp)
target_link_libraries(sim_ipc
  PUBLIC sim_runtime sim_protocol
  PRIVATE sim_build_options)

# -----------------------------------------------------------------------------
# App: the vehicles and flight controllers the programs set up from the options
# -----------------------------------------------------------------------------

add_library(sim_app STATIC src/app/setup.cpp)
target_link_libraries(sim_app
  PUBLIC sim_ipc sim_runtime
  PRIVATE sim_build_options sim_control)

# -----------------------------------------------------------------------------
# Headless executable: no window, no GPU
# -----------------------------------------------------------------------------

add_executable(multirotor-sim-headless src/headless/main.cpp)
target_link_libraries(multirotor-sim-headless PRIVATE sim_build_options sim_app sim_control sim_ipc)

# -----------------------------------------------------------------------------
# Command-line tool sending requests to a running simulator
# -----------------------------------------------------------------------------

add_executable(multirotor-sim-ctl src/ctl/main.cpp)
target_link_libraries(multirotor-sim-ctl PRIVATE sim_build_options sim_ipc)

# -----------------------------------------------------------------------------
# Example SITL flight controller, in C
# -----------------------------------------------------------------------------

add_executable(sitl-example-client examples/sitl_client.c)
target_link_libraries(sitl-example-client PRIVATE sim_build_options sim_protocol m)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------

enable_testing()
# Each simulator claims a socket directory, so each test gets its own.
# Socket paths are limited to 107 bytes, so keep these short.
set(test_sockets ${CMAKE_CURRENT_BINARY_DIR}/sockets)
add_executable(sensor-test tests/sensor_test.cpp)
target_link_libraries(sensor-test PRIVATE sim_build_options sim_core)
add_test(NAME sensors COMMAND sensor-test)
add_executable(sim-server-test tests/sim_server_test.cpp)
target_link_libraries(sim-server-test PRIVATE sim_build_options sim_ipc)
add_test(NAME sim-server COMMAND sim-server-test ${test_sockets}/sim-server)
# The headless executable flies a scripted flight and fails if the state diverges.
add_test(NAME headless-quadrotor COMMAND multirotor-sim-headless --socket-dir ${test_sockets}/quadrotor)
add_test(NAME headless-hexarotor
  COMMAND multirotor-sim-headless --rotors 6 --socket-dir ${test_sockets}/hexarotor)
add_test(NAME headless-octorotor
  COMMAND multirotor-sim-headless --rotors 8 --mass 2.5 --socket-dir ${test_sockets}/octorotor)
add_test(NAME headless-three-vehicles
  COMMAND multirotor-sim-headless --vehicles 3 --socket-dir ${test_sockets}/three-vehicles)
# Lockstep over the SITL socket: the example client flies a circle and fails if it
# tracks it poorly. The simulator runs in the background and ends the run.
add_test(NAME sitl-lockstep
  COMMAND sh -c [["$1" --sitl --socket-dir "$3" --duration 20 >/dev/null & simulator=$!
                  "$2" --socket-dir "$3"; client=$?
                  wait $simulator && exit $client]]
    sh $<TARGET_FILE:multirotor-sim-headless> $<TARGET_FILE:sitl-example-client>
    ${test_sockets}/sitl)
# Vehicles spawned and removed while the simulator runs, through its socket.
add_test(NAME spawn-and-remove
  COMMAND sh -c [[tool=$2 sockets=$3
                  "$1" --realtime --duration 30 --socket-dir "$sockets" >/dev/null & simulator=$!
                  ctl() { "$tool" --socket-dir "$sockets" "$@"; }
                  sleep 0.5
                  ctl spawn alpha --at 0,5 --rotors 6 &&
                  ! ctl spawn alpha 2>/dev/null &&
                  ctl spawn bravo --sitl --at 0,-5 && [ -S "$sockets/vehicles/bravo.sock" ] &&
                  ctl remove bravo && [ ! -e "$sockets/vehicles/bravo.sock" ] &&
                  ctl remove alpha && ! ctl remove alpha 2>/dev/null &&
                  ! ctl spawn charlie --rotors 5 2>/dev/null &&
                  ctl speed 2 && ctl pause && ctl resume && ctl reset; result=$?
                  kill $simulator; wait $simulator; exit $result]]
    sh $<TARGET_FILE:multirotor-sim-headless> $<TARGET_FILE:multirotor-sim-ctl> ${test_sockets}/spawn)
# Two vehicles in lockstep, each with its own flight controller.
add_test(NAME sitl-two-vehicles
  COMMAND sh -c [["$1" --sitl --vehicle alpha --vehicle bravo --socket-dir "$3" --duration 20 >/dev/null &
                  simulator=$!
                  "$2" --socket-dir "$3" --vehicle alpha >/dev/null & alpha=$!
                  "$2" --socket-dir "$3" --vehicle bravo; bravo=$?
                  wait $alpha; alpha=$?
                  wait $simulator && [ $alpha -eq 0 ] && exit $bravo]]
    sh $<TARGET_FILE:multirotor-sim-headless> $<TARGET_FILE:sitl-example-client>
    ${test_sockets}/pair)
set_tests_properties(sitl-lockstep PROPERTIES TIMEOUT 60)

# -----------------------------------------------------------------------------
# Wayland protocol bindings (generated C code)
# -----------------------------------------------------------------------------

add_library(sim_wayland_protocols STATIC)
wayland_generate_protocols(sim_wayland_protocols
  stable/xdg-shell/xdg-shell.xml
  stable/viewporter/viewporter.xml
  unstable/xdg-decoration/xdg-decoration-unstable-v1.xml
  staging/fractional-scale/fractional-scale-v1.xml
  unstable/tablet/tablet-unstable-v2.xml # Required by cursor-shape-v1.
  staging/cursor-shape/cursor-shape-v1.xml)
target_link_libraries(sim_wayland_protocols PUBLIC PkgConfig::wayland-client)
set_target_properties(sim_wayland_protocols PROPERTIES C_STANDARD 23 C_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Platform layer: Wayland connection, window and input
# -----------------------------------------------------------------------------

add_library(sim_platform STATIC
  src/platform/wayland/display.cpp
  src/platform/wayland/handle.cpp
  src/platform/wayland/seat.cpp
  src/platform/wayland/window.cpp)
target_include_directories(sim_platform PUBLIC src)
target_link_libraries(sim_platform
  PUBLIC PkgConfig::xkbcommon
  PRIVATE sim_build_options sim_wayland_protocols PkgConfig::wayland-client)

# -----------------------------------------------------------------------------
# Shaders, compiled to SPIR-V and embedded in the program
# -----------------------------------------------------------------------------

add_shader_library(sim_shaders
  shaders/mesh.vert
  shaders/mesh.frag)

# -----------------------------------------------------------------------------
# Render layer: Vulkan device, swapchain and frame loop
# -----------------------------------------------------------------------------

add_library(sim_render STATIC
  src/render/memory.cpp
  src/render/offscreen_renderer.cpp
  src/render/renderer.cpp
  src/render/scene_pass.cpp
  src/render/swapchain.cpp
  src/render/vulkan_context.cpp)
target_include_directories(sim_render PUBLIC src)
target_link_libraries(sim_render
  PUBLIC Vulkan::Headers PkgConfig::wayland-client
  PRIVATE sim_build_options sim_shaders)
target_compile_definitions(sim_render PUBLIC
  VK_NO_PROTOTYPES
  VK_USE_PLATFORM_WAYLAND_KHR
  VULKAN_HPP_DISPATCH_LOADER_DYNAMIC=1
  VULKAN_HPP_NO_CONSTRUCTORS # Enables designated initializers for Vulkan-Hpp structs.
  VULKAN_HPP_HANDLE_ERROR_OUT_OF_DATE_AS_SUCCESS) # Swapchain out-of-date is a result, not an exception.
# vulkan_raii.hpp is large; precompiling it keeps incremental builds fast.
target_precompile_headers(sim_render PRIVATE <vulkan/vulkan_raii.hpp>)

# -----------------------------------------------------------------------------
# Scene: what is drawn for a snapshot, shared by the viewer and the recorder
# -----------------------------------------------------------------------------

add_library(sim_scene STATIC
  src/scene/meshes.cpp
  src/scene/scene_view.cpp
  src/scene/transform.cpp)
target_link_libraries(sim_scene
  PUBLIC sim_ipc sim_runtime
  PRIVATE sim_build_options)

# -----------------------------------------------------------------------------
# Viewer executable: simulation thread plus a window showing it
# -----------------------------------------------------------------------------

add_executable(multirotor-sim
  src/viewer/main.cpp
  src/viewer/viewer.cpp)
target_link_libraries(multirotor-sim
  PRIVATE sim_build_options sim_app sim_control sim_ipc sim_platform sim_render sim_scene)
# Reuse sim_render's precompiled header instead of building a second copy.
target_precompile_headers(multirotor-sim REUSE_FROM sim_render)

# -----------------------------------------------------------------------------
# Recorder executable: renders a flight offscreen to a video file, via ffmpeg
# -----------------------------------------------------------------------------

add_executable(multirotor-sim-record
  src/recorder/main.cpp
  src/recorder/video_encoder.cpp)
target_link_libraries(multirotor-sim-record
  PRIVATE sim_build_options sim_app sim_control sim_ipc sim_render sim_scene)
target_precompile_headers(multirotor-sim-record REUSE_FROM sim_render)
