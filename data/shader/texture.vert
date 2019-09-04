#version 330
in vec3 position;
out vec2 texcoord;

void main() {
    texcoord = 0.5 * position.xy + vec2(0.5, 0.5);
    gl_Position = vec4(position, 1.0);
}