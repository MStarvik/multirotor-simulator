#version 460

layout(push_constant) uniform PushConstants {
    mat4 view_projection;
    mat4 model;
} pc;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;

layout(location = 0) out vec3 out_normal;  // World space.
layout(location = 1) out vec3 out_color;

void main() {
    gl_Position = pc.view_projection * pc.model * vec4(in_position, 1.0);
    // Model transforms are rigid, so the upper 3x3 transforms normals too.
    out_normal = mat3(pc.model) * in_normal;
    out_color = in_color;
}
