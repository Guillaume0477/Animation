#version 330 core

in vec2 v_uv;
in vec3 v_vit;
in vec3 v_force;

uniform sampler2D tex;

out vec4 color;

void main()
{
    color = texture(tex, v_uv); 
};
