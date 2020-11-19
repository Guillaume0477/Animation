/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mesh_parametric_cloth.hpp"

#include "../lib/common/error_handling.hpp"
#include <cmath>

namespace cpe
{


cpe::vec3 getElasticForce(cpe::vec3 origin, cpe::vec3 neighbour, float K = 20.0f, float L_rest = 1.0/30.0f){

    cpe::vec3 u = neighbour - origin;
    float L = norm(u);
    cpe::vec3 F = K*(L - L_rest)*u/L;

    return F;
}

cpe::vec3 mesh_parametric_cloth::getStructuralForce(int ku, int kv, int const Nu, int const Nv, float K_structural){

    //Structural forces
    cpe::vec3 Fright, Fleft, Ftop, Fbottom = cpe::vec3(0.0,0.0,0.0);
    cpe::vec3 curVec = vertex(ku,kv);
    float L_structural = 1.0/Nu;
    if (ku + 1 < Nu){
        Fright = getElasticForce(curVec, vertex(ku+1,kv), K_structural, L_structural);
    }
    if (ku - 1 >= 0){
        Fleft = getElasticForce(curVec, vertex(ku-1, kv), K_structural, L_structural);
    }
    if (kv + 1 < Nv){
        Fbottom = getElasticForce(curVec, vertex(ku, kv+1), K_structural, L_structural);
    }
    if (kv - 1 >= 0){
        Ftop = getElasticForce(curVec, vertex(ku, kv-1), K_structural, L_structural);
    }
    
    return (Ftop + Fright + Fbottom + Fleft);

}

cpe::vec3 mesh_parametric_cloth::getBendingForce(int ku, int kv, int const Nu, int const Nv, float K_bend){

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


cpe::vec3 mesh_parametric_cloth::getShearingForce(int ku, int kv, int const Nu, int const Nv, float K_shearing){

    //Structural forces
    cpe::vec3 Ftopright, FbottomLeft, FtopLeft, FbottomRight = cpe::vec3(0.0,0.0,0.0);
    cpe::vec3 curVec = vertex(ku,kv);
    float L_Shear = float(sqrt(2))/Nu;
    if ((ku + 1 < Nu) && (kv + 1 < Nv)) {
        Ftopright = getElasticForce(curVec, vertex(ku+1,kv+1), K_shearing, L_Shear);
    }
    if ((ku - 1 >= 0) && (kv - 1 >= 0)){
        FbottomLeft = getElasticForce(curVec, vertex(ku-1, kv-1), K_shearing, L_Shear);
    }
    if ((ku - 1 >= 0) && (kv + 1 < Nv)){
        FtopLeft = getElasticForce(curVec, vertex(ku-1, kv+1), K_shearing, L_Shear);
    }
    if ((ku + 1 < Nu) && (kv - 1 >= 0)){
        FbottomRight = getElasticForce(curVec, vertex(ku+1, kv-1), K_shearing, L_Shear);
    }
    
    return (Ftopright + FtopLeft + FbottomRight + FbottomLeft);

}


cpe::vec3 getWindForce(cpe::vec3 normalVec, float K, cpe::vec3 u){
    float cosTheta = dot(normalVec, u);
    // std::cout << cosTheta << std::endl;
    cpe::vec3 f = K*cosTheta*normalVec;
    // std::cout << f << std::endl;
    return f;
}


void mesh_parametric_cloth::update_force()
{

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(force_data.size()) == Nu*Nv , "Error of size");

    cpe::vec3 dir_wind = cpe::vec3(0.0, 1.0, 0.0);

    //Gravity
    static vec3 const g (0.0f,0.0f,-9.81f);
    vec3 const g_normalized = g/N_total;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            force(ku,kv) = g_normalized;
        }
    }

    //*************************************************************//
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//
    
    //Fix forces of the corners to 0
    force(0,0) = vec3(0.0,0.0,0.0);
    force(Nu-1,0) = vec3(0.0,0.0,0.0);

    float K_structural = 20.0;
    float K_shearing = 8.0;
    float K_bending = 8.0f;
    float K_wind = 0.01f;

    for (int ku = 0 ; ku < Nu ; ++ku){
        for (int kv = 0 ; kv < Nv ; ++kv){
            if (((ku == 0) && (kv == 0)) || ((kv == 0) && (ku == Nu-1))){
                continue;
            }
            force(ku,kv) += getStructuralForce(ku, kv, Nu, Nv, K_structural);
            force(ku,kv) += getShearingForce(ku, kv, Nu, Nv, K_structural);
            force(ku,kv) += getBendingForce(ku, kv, Nu, Nv, K_bending);
            
            force(ku, kv) += getWindForce(normal(ku,kv), K_wind, dir_wind);

        }
    }

    collisionPlan(2, -1.1);
    collisionSphere(0.198f , {0.5f,0.05f,-1.1f});
    

    //*************************************************************//


}

void mesh_parametric_cloth::integration_step(float const dt)
{
    ASSERT_CPE(speed_data.size() == force_data.size(),"Incorrect size");
    ASSERT_CPE(static_cast<int>(speed_data.size()) == size_vertex(),"Incorrect size");


    int const Nu = size_u();
    int const Nv = size_v();
    //*************************************************************//
    // TO DO: Calculer l'integration numerique des positions au cours de l'intervalle de temps dt.
    //*************************************************************//
    float damping = 0.2;
    for (int i = 0; i < Nu*Nv; i++){
        speed_data[i] = (1-damping * dt) * speed_data[i] + dt*force_data[i];
        vertex_data[i] += dt*speed_data[i];
    }

    //*************************************************************//


    //security check (throw exception if divergence is detected)
    static float const LIMIT=30.0f;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            vec3 const& p = vertex(ku,kv);

            if( norm(p) > LIMIT )
            {
                throw exception_divergence("Divergence of the system",EXCEPTION_PARAMETERS_CPE);
            }
        }
    }

}

void mesh_parametric_cloth::set_plane_xy_unit(int const size_u_param,int const size_v_param)
{
    mesh_parametric::set_plane_xy_unit(size_u_param,size_v_param);

    int const N = size_u()*size_v();
    speed_data.resize(N);
    force_data.resize(N);
}

vec3 const& mesh_parametric_cloth::speed(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3& mesh_parametric_cloth::speed(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3 const& mesh_parametric_cloth::force(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}

vec3& mesh_parametric_cloth::force(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}


void mesh_parametric_cloth::collisionPlan(int axis, float limit){

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;

    for (int k = 0; k < N_total ; k++){
        if (vertex_data[k][axis] < limit + 0.02){
            // vertex_data[k][axis] = limit+0.02;
            if (force_data[k][axis] < 0){
                force_data[k][axis] = 0;
            }
            if (speed_data[k][axis] < 0){
                speed_data[k][axis] = 0;
            }
        }
    }

}

void mesh_parametric_cloth::collisionSphere(float radius, vec3 center){

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;

    for (int k = 0; k < N_total ; k++){
        vec3 vectorDir = vertex_data[k]-center;
        float value = norm(vectorDir);

        if ( value < radius + 0.01){
            vectorDir /= value;
            float projF = dot(force_data[k], vectorDir);
            vec3 revF = projF * vectorDir;
            if (projF < 0)
            {
                force_data[k] -= revF;
            }

            projF = dot(speed_data[k], vectorDir);
            revF = projF * vectorDir;
            if (projF < 0)
            {
                speed_data[k] -= revF;
            }

        }
    }


}

}

