#version 330
in vec3 vert_position;
out vec2 texcoord;

void main() {
    texcoord = 0.5 * vert_position.xy + vec2(0.5, 0.5);
    gl_Position = vec4(vert_position, 1.0);
}