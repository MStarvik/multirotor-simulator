#version 460

layout(location = 0) in vec3 in_normal;
layout(location = 1) in vec3 in_color;
layout(location = 2) in float in_view_depth;

layout(location = 0) out vec4 out_color;

// Direction towards the sun in the world frame (NED: -z is up).
const vec3 sun_direction = normalize(vec3(-0.4, 0.3, -1.0));
const float ambient = 0.35;

// Distant surfaces fade into the sky, which hides the edge of the finite ground.
// The color must match sky_color in src/scene/scene_view.cpp.
const vec3 fog_color = vec3(0.42, 0.60, 0.85);
const float fog_start = 40.0;  // m
const float fog_end = 80.0;    // m

void main() {
    // Two-sided lighting, since faces are drawn from both sides.
    const float diffuse = abs(dot(normalize(in_normal), sun_direction));
    const vec3 lit = in_color * (ambient + (1.0 - ambient) * diffuse);
    const float fog = smoothstep(fog_start, fog_end, in_view_depth);
    out_color = vec4(mix(lit, fog_color, fog), 1.0);
}
