include_guard(GLOBAL)

# Locates wayland-scanner and the wayland-protocols XML files, and provides
# wayland_generate_protocols() to turn protocol XML into C bindings.

find_package(PkgConfig REQUIRED)

pkg_check_modules(wayland-protocols REQUIRED wayland-protocols>=1.38)
pkg_get_variable(WAYLAND_PROTOCOLS_DIR wayland-protocols pkgdatadir)

pkg_check_modules(wayland-scanner REQUIRED wayland-scanner)
pkg_get_variable(_wayland_scanner_path wayland-scanner wayland_scanner)
find_program(WAYLAND_SCANNER_EXECUTABLE
  NAMES wayland-scanner
  HINTS "${_wayland_scanner_path}"
  REQUIRED)

# wayland_generate_protocols(<target> <protocol.xml>...)
#
# Generates the client header and glue code for each protocol and adds them to
# <target>. Relative paths are resolved against the wayland-protocols data
# directory, e.g. "stable/xdg-shell/xdg-shell.xml". The generated headers are
# exposed as system includes so consumers' warning flags don't apply to them.
function(wayland_generate_protocols target)
  set(out_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}")
  # Ninja creates output directories itself, but the Makefile generator doesn't.
  file(MAKE_DIRECTORY "${out_dir}")

  foreach(xml IN LISTS ARGN)
    if(NOT IS_ABSOLUTE "${xml}")
      set(xml "${WAYLAND_PROTOCOLS_DIR}/${xml}")
    endif()
    cmake_path(GET xml STEM LAST_ONLY name)

    set(header "${out_dir}/${name}-client-protocol.h")
    set(code "${out_dir}/${name}-protocol.c")

    add_custom_command(
      OUTPUT "${header}"
      COMMAND "${WAYLAND_SCANNER_EXECUTABLE}" client-header "${xml}" "${header}"
      DEPENDS "${xml}"
      COMMENT "Generating Wayland client header for ${name}"
      VERBATIM)
    add_custom_command(
      OUTPUT "${code}"
      COMMAND "${WAYLAND_SCANNER_EXECUTABLE}" private-code "${xml}" "${code}"
      DEPENDS "${xml}"
      COMMENT "Generating Wayland protocol code for ${name}"
      VERBATIM)

    target_sources(${target} PRIVATE "${header}" "${code}")
  endforeach()

  target_include_directories(${target} SYSTEM PUBLIC "${out_dir}")
endfunction()
