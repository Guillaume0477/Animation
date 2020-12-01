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
    vec3 u = vec3(pos[xy2i(n)] - pos[xy2i(p)]);
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

    float L_structural = 2.0f/N;
    
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

vec4 getBendingForce(in uvec2 p, in float K_bend){

    //Structural forces
    int ku = int(p.x);
    int kv = int(p.y);
    vec4 F = vec4(0.0,0.0,0.0,0.0);
 
    float L_Bending = 4.0f/N;
    if (ku + 2 < N){
        F += compute_spring_force(p,uvec2(ku+2,kv),K_bend,L_Bending);
    }
    if (ku - 2 >= 0){
        F += compute_spring_force(p,uvec2(ku-2,kv),K_bend,L_Bending);
    }
    if (kv + 2 < N){
        F += compute_spring_force(p,uvec2(ku,kv+2),K_bend,L_Bending);
    }
    if (kv - 2 >= 0){
        F += compute_spring_force(p,uvec2(ku,kv-2),K_bend,L_Bending);
    }
    
    return (F);

}


vec4 getShearingForce(in uvec2 p, in float K_shearing){

    //Structural forces
    int ku = int(p.x);
    int kv = int(p.y);
    vec4 F= vec4(0.0,0.0,0.0,0.0);
 
    float L_Shear = 2*sqrt(2.0f)/N;
    if ((ku + 1 < N) && (kv + 1 < N)) {
        F += compute_spring_force(p,uvec2(ku+1,kv+1),K_shearing,L_Shear);
    }
    if ((ku - 1 >= 0) && (kv - 1 >= 0)){
        F += compute_spring_force(p,uvec2(ku-1,kv-1),K_shearing,L_Shear);
    }
    if ((ku - 1 >= 0) && (kv + 1 < N)){
        F += compute_spring_force(p,uvec2(ku-1,kv+1),K_shearing,L_Shear);
    }
    if ((ku + 1 < N) && (kv - 1 >= 0)){
        F += compute_spring_force(p,uvec2(ku+1,kv-1),K_shearing,L_Shear);
    }
    

    return (F);

}


vec4 getWindForce(uvec2 p, float K, vec3 u){


     //Structural forces
    int ku = int(p.x);
    int kv = int(p.y);
    vec4 F= vec4(0.0,0.0,0.0,0.0);
    vec3 normalVec = vec3(0.0, 0.0, 0.0);
    
    vec4 p0 = pos[xy2i(uvec2(ku,kv))];
    vec4 p1 = vec4(0.0, 0.0, 0.0, 0.0);
    vec4 p2 = vec4(0.0, 0.0, 0.0, 0.0);
    
    if (ku + 1 < N) {
        p1 = pos[xy2i(uvec2(ku+1, kv))];
    } else {
        p1 = pos[xy2i(uvec2(ku-1,kv))];
    }

    if (kv + 1 < N) {
        p2 = pos[xy2i(uvec2(ku, kv+1))];
    } else {
        p2 = pos[xy2i(uvec2(ku,kv-1))];
    }

    normalVec = normalize(cross(vec3(p1-p0),vec3(p2-p0)));

    float cosTheta = dot(normalVec, u);
    // std::cout << cosTheta << std::endl;
    F = vec4(K*cosTheta*normalVec,0.0f);
    // std::cout << f << std::endl;
    return F;
}



void main() {
    uvec2 id_2d = gl_GlobalInvocationID.xy;
    uint id = xy2i(id_2d);
    
    //*************************************************************// 
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//
    //
    float K_structural = 20.0f;
    float K_shearing = 8.0f;
    float K_bending = 8.0f;
    float K_wind = 0.01f;
    vec3 u = vec3(1.0, 0.0, 0.0);

    if (id == xy2i(uvec2(0,0)) || (id == xy2i(uvec2(N-1,0)))) {
        force[id] = vec4(0.0,0.0,0.0,0.0);
    }
    else{
        force[id] = vec4(0.0,-9.81,0.0,0.0)/900.0;//
        force[id] += getStructuralForce(id_2d,K_structural);
        force[id] += getBendingForce(id_2d,K_bending);
        force[id] += getShearingForce(id_2d,K_shearing);

        force[id] += getWindForce(id_2d, K_wind, u);
    }

    //
    //
    //*************************************************************//
}
