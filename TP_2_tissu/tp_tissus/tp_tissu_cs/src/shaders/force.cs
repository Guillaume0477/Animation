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

vec4 compute_spring_force(in uvec2 p, in uvec2 n, in float K, in float L0)
{
    //*************************************************************//
    // TO DO, Calculer la force s'appliquant du ressort entre p et n
    //*************************************************************//

    //if n.x < 0 ou >
    //pareil sur y
    vec3 u = vec3(pos[xy2i(p)] + pos[xy2i(n)]);
    //vec3 u = vec3(0,1,0);

    float L = sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
    vec3 F = K*(L - L0)*u/L;
    


    //*************************************************************//
    return vec4(F, 0.0f);
}

vec4 getStructuralForce( in uvec2 p, in float K_structural){

    //Structural forces
    int ku = int(p.x);
    int kv = int(p.y);
    vec4 F = vec4(0.0,0.0,0.0,0.0);

    float L_structural = 1.0f/N;
    
    if (ku + 1 < N){
        F += compute_spring_force(p,uvec2(ku+1,kv),K_structural,L_structural);
    }
    if (ku - 1 >= 0){
        F += compute_spring_force(p,uvec2(ku-1,kv),K_structural,L_structural);
    }
    if (kv + 1 < N){
        F += compute_spring_force(p,uvec2(ku,kv+1),K_structural,L_structural);
    }
    if (kv - 1 >= 0){
        F += compute_spring_force(p,uvec2(ku,kv-1),K_structural,L_structural);
    }

    return (F);

}
/*
vec3 mesh_parametric_cloth::getBendingForce(int ku, int kv, int const Nu, int const Nv, float K_bend){

    //Structural forces
    cpe::vec3 Fright, Fleft, Ftop, Fbottom = cpe::vec3(0.0,0.0,0.0);
    cpe::vec3 curVec = vertex(ku,kv);
    float L_Bending = 2.0/Nu;
    if (ku + 2 < Nu){
        Fright = getElasticForce(curVec, vertex(ku+2,kv), K_bend, L_Bending);
    }
    if (ku - 2 >= 0){
        Fleft = getElasticForce(curVec, vertex(ku-2, kv), K_bend, L_Bending);
    }
    if (kv + 2 < Nv){
        Fbottom = getElasticForce(curVec, vertex(ku, kv+2), K_bend, L_Bending);
    }
    if (kv - 2 >= 0){
        Ftop = getElasticForce(curVec, vertex(ku, kv-2), K_bend, L_Bending);
    }
    
    return (Ftop + Fright + Fbottom + Fleft);

}

*/
// vec3 mesh_parametric_cloth::getShearingForce(int ku, int kv, int const Nu, int const Nv, float K_shearing){

//     //Structural forces
//     cpe::vec3 Ftopright, FbottomLeft, FtopLeft, FbottomRight = cpe::vec3(0.0,0.0,0.0);
//     cpe::vec3 curVec = vertex(ku,kv);
//     float L_Shear = float(sqrt(2))/Nu;
//     if ((ku + 1 < Nu) && (kv + 1 < Nv)) {
//         Ftopright = getElasticForce(curVec, vertex(ku+1,kv+1), K_shearing, L_Shear);
//     }
//     if ((ku - 1 >= 0) && (kv - 1 >= 0)){
//         FbottomLeft = getElasticForce(curVec, vertex(ku-1, kv-1), K_shearing, L_Shear);
//     }
//     if ((ku - 1 >= 0) && (kv + 1 < Nv)){
//         FtopLeft = getElasticForce(curVec, vertex(ku-1, kv+1), K_shearing, L_Shear);
//     }
//     if ((ku + 1 < Nu) && (kv - 1 >= 0)){
//         FbottomRight = getElasticForce(curVec, vertex(ku+1, kv-1), K_shearing, L_Shear);
//     }
    
//     return (Ftopright + FtopLeft + FbottomRight + FbottomLeft);

// }






void main() {
    uvec2 id_2d = gl_GlobalInvocationID.xy;
    uint id = xy2i(id_2d);
    
    //*************************************************************// 
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//
    //
    float K_structural = 10.0;
    force[id] = vec4(0.0,-9.81,0.0,0.0)/900;//
    force[id] += getStructuralForce(id_2d,K_structural);
    //force[id] += compute_spring_force(id_2d,uvec2(0,1),0.1f,0.1f);;
    //compute_spring_force(id_2d, ivec2(id_2d) + ivec2(0,1) , float K, float L0)
    //
    //
    //
    //*************************************************************//
}
