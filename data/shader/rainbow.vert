#version 330
uniform float point_scale;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

uniform int color_mode;
uniform vec4 material_color;

uniform vec2 z_range;

in vec3 vert_position;
out vec4 frag_color;
out vec3 frag_world_position;

vec3 turbo(in float x) {
    const vec4 kRedVec4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 kGreenVec4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 kBlueVec4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kRedVec2 = vec2(-152.94239396, 59.28637943);
    const vec2 kGreenVec2 = vec2(4.27729857, 2.82956604);
    const vec2 kBlueVec2 = vec2(-89.90310912, 27.34824973);

    x = clamp(x, 0.0, 1.0);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z;
    return vec3(
        dot(v4, kRedVec4)   + dot(v2, kRedVec2),
        dot(v4, kGreenVec4) + dot(v2, kGreenVec2),
        dot(v4, kBlueVec4)  + dot(v2, kBlueVec2)
    );
}

vec4 rainbow(vec3 position) {
    float p = (position.z - z_range[0]) / (z_range[1] - z_range[0]);
    return vec4(turbo(p), 1.0);
}

void main() {
    vec4 world_position = model_matrix * vec4(vert_position, 1.0);
    frag_world_position = world_position.xyz;
    gl_Position = projection_matrix * view_matrix * world_position;

    if(color_mode == 0) {
        frag_color = rainbow(frag_world_position);
    } else if(color_mode == 1) {
        frag_color = material_color;
    }

    vec3 ndc = gl_Position.xyz / gl_Position.w;
    float z_dist = 1.0 - ndc.z;
    gl_PointSize = point_scale * z_dist;
}