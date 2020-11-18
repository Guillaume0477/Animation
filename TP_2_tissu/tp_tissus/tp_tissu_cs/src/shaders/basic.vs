#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 vitesse;
layout (location = 2) in vec3 force;
layout (location = 3) in vec2 uv;


uniform mat4 MVP;

out vec3 v_pos;
out vec2 v_uv;
out vec3 v_vit;
out vec3 v_force;

void main()
{
    gl_Position = MVP * vec4(position, 1.0);
    v_pos = position;
    v_uv = uv;
    v_vit = vitesse;
    v_force = force;
};
