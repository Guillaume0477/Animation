#version 430

layout(local_size_x = 1, local_size_y = 1) in;

layout(std430, binding = 1) buffer positionBlock
{
    vec4 pos[];
};

layout(std430, binding = 2) buffer speedBlock
{
    vec4 vit[];
};

layout(std430, binding = 3) buffer forceBlock
{
    vec4 force[];
};

uniform uint N;
uniform float dt;

float mu = 0.2;

uint xy2i(uvec2 id_2d)
{
    return id_2d.x * N + id_2d.y;
}

void main() {
    uvec2 id_2d = gl_GlobalInvocationID.xy;
    uint id = xy2i(id_2d);

    //*************************************************************//
    // TO DO, Calculer les nouveaux parametre vit et pos 
    // selon l integration d euler explicite
    //*************************************************************//
    //
    // ...
    //
    //*************************************************************//
}
