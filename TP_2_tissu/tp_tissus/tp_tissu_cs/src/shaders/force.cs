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

uint xy2i(uvec2 id_2d)
{
    return id_2d.x * N + id_2d.y;
}

vec3 compute_spring_force(uvec2 p, ivec2 n, float K, float L0)
{
    //*************************************************************//
    // TO DO, Calculer la force s'appliquant du ressort entre p et n
    //*************************************************************//
    //
    // ...
    //
    //
    //
    //*************************************************************//
    return vec3(0.0f, 0.0f, 0.0f);
}

void main() {
    uvec2 id_2d = gl_GlobalInvocationID.xy;
    uint id = xy2i(id_2d);
    
    //*************************************************************//
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//
    //
    // ...
    //
    //
    //
    //*************************************************************//
}
