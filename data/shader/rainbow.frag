#version 330
uniform vec2 z_range;
uniform ivec4 info_values;

in vec4 frag_color;
in vec3 frag_world_position;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;

void main() {
    if(frag_world_position.z < z_range[0] || frag_world_position.z > z_range[1]) {
        discard;
    }

    color = frag_color;
    info = info_values;
}