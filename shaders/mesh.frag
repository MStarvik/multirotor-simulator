#version 460

layout(location = 0) in vec3 in_normal;
layout(location = 1) in vec3 in_color;

layout(location = 0) out vec4 out_color;

// Direction towards the sun in the world frame (NED: -z is up).
const vec3 sun_direction = normalize(vec3(-0.4, 0.3, -1.0));
const float ambient = 0.35;

void main() {
    // Two-sided lighting, since faces are drawn from both sides.
    const float diffuse = abs(dot(normalize(in_normal), sun_direction));
    out_color = vec4(in_color * (ambient + (1.0 - ambient) * diffuse), 1.0);
}
