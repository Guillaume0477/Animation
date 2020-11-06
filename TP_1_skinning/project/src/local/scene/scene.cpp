

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"

#include <cmath>


#include <string>
#include <sstream>
#include "../../lib/mesh/mesh_io.hpp"



using namespace cpe;



static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex(vec3(-L, h,-L));
    m.add_vertex(vec3(-L, h, L));
    m.add_vertex(vec3( L, h, L));
    m.add_vertex(vec3( L, h,-L));

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}



static cpe::mesh_skinned build_cylinder(float const R,float const H, int const nbT, int const nbH) //rayon hauteur
{
    mesh_skinned m;

    float incT = 2*M_PI / nbT;
    float incH = H / nbH;

    //Create vertice of the cylinder
    for (int k = 0; k < nbH; k++){
        float h = k*incH;
        for (int i = 0; i < nbT ; i++){
            vertex_weight_parameter vwp;
            skinning_weight sw, swbis;

            float theta = i*incT;
            m.add_vertex(vec3(R*cos(theta), R*sin(theta), h));


            // //utilisation d'un poids soit 0 soit 1 strict
            // if (h>H/2){
            //     sw.joint_id = 1;
            //     sw.weight = 1;
            // } else {
            //     sw.joint_id = 0;
            //     sw.weight = 1;
            // }
            // vwp[0] = sw; 

            //utilisation d'un poids linéaire
            sw.joint_id = 0;
            sw.weight = 1 - h/H;
            swbis.joint_id = 1;
            swbis.weight = h/H;
            vwp[0] = sw; 
            vwp[1] = swbis;
 
            m.add_vertex_weight(vwp);
        }
    }
    //Creates triangles of the cylinder
    for (int i = 0; i < nbH-1; i++){
        for (int j = 0; j < nbT-1; j++){
            
            m.add_triangle_index({i*nbT + j, i*nbT + j+1, (i+1)*nbT + j + 1});
            m.add_triangle_index({i*nbT + j , (i+1)*nbT + j+1, (i+1)*nbT + j});
        }

        m.add_triangle_index({(i+1)*nbT-1, i*nbT, (i+1)*nbT});
        m.add_triangle_index({(i+1)*nbT-1, (i+1)*nbT, (i+2)*nbT-1});
    }

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}


static void Init_cylinder_skeleton(cpe::skeleton_parent_id &sk_cylinder_parent_id,cpe::skeleton_geometry &sk_cylinder_bind_pose ,float length) //quaternion q
{
    sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);

    skeleton_joint j00 = skeleton_joint(vec3(0,0,0),cpe::quaternion(0,0,0,1));
    skeleton_joint j11 = skeleton_joint(vec3(0,0,length/2),cpe::quaternion(0,0,0,1));
    skeleton_joint j22 = skeleton_joint(vec3(0,0,length/2),cpe::quaternion(0,0,0,1));

    sk_cylinder_bind_pose.push_back(j00);
    sk_cylinder_bind_pose.push_back(j11);
    sk_cylinder_bind_pose.push_back(j22);

}

static void Init_cylinder_animation(cpe::skeleton_parent_id &sk_cylinder_parent_id,cpe::skeleton_geometry sk_cylinder_bind_pose ,cpe::skeleton_animation &sk_cylinder_animation ,float length) //quaternion q
{

    //for 0 to 90 pas 30
    //new ske_geo

    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    skeleton_joint j_30 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_30.orientation.set_axis_angle(vec3(1,0,0),M_PI/6);
    sk_cylinder_bind_pose[1] = j_30;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    skeleton_joint j_60 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_60.orientation.set_axis_angle(vec3(1,0,0),M_PI/3);
    sk_cylinder_bind_pose[1] = j_60;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    skeleton_joint j_90 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_90.orientation.set_axis_angle(vec3(1,0,0),M_PI/2);
    sk_cylinder_bind_pose[1] = j_90;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);


}

void scene::load_scene()
{

    float const length = 50.0f;
    float const radius = 4.0f;
    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");           PRINT_OPENGL_ERROR();
    shader_skeleton = read_shader("shaders/shader_skeleton.vert",
                                  "shaders/shader_skeleton.frag");       PRINT_OPENGL_ERROR();


    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(100.0f , -25.0f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);


    //*****************************************//
    // Build cylinder
    //*****************************************//
    mesh_cylinder = build_cylinder(radius, length, 40,40);
    mesh_cylinder.fill_empty_field_by_default();
    mesh_cylinder_opengl.fill_vbo(mesh_cylinder);

    Init_cylinder_skeleton(sk_cylinder_parent_id, sk_cylinder_bind_pose , length) ;

    //Test to check if the function works
    // cpe::skeleton_geometry glob = local_to_global(sk_cylinder_bind_pose, sk_cylinder_parent_id);

    Init_cylinder_animation(sk_cylinder_parent_id, sk_cylinder_bind_pose, sk_cylinder_animation, length);
    



    time.start();
}



void scene::draw_scene()
{
    setup_shader_skeleton(shader_skeleton);

    if (time.elapsed()>1000){
        index = next;

        if ((order > 0) && (next + 1 < sk_cylinder_animation.size())){
            next +=1;
        } else if ((order > 0) && (next + 1 >= sk_cylinder_animation.size())){
            order *= -1;
            next -= 1;
        } else if ((order < 0) && (next - 1 >= 0)){
            next -=1;
        } else {
            order *= -1;
            next +=1;
        }
        time.restart();

    }

    float alpha = float(time.elapsed()) / 1000.0;

    skeleton_geometry sk_toUse = interpolated(sk_cylinder_animation[index], sk_cylinder_animation[next], alpha); //sk_cylinder_animation(index, alpha);// sk_cylinder_animation[index]; //


    skeleton_geometry const sk_cylinder_global = local_to_global(sk_toUse,sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
    draw_skeleton(sk_cylinder_bones);

    //Here we can draw skeletons as 3D segments

    setup_shader_mesh(shader_mesh);

    mesh_ground_opengl.draw();

    skeleton_geometry const sk_cylinder_inverse_bind_pose = inversed(sk_cylinder_bind_pose);
    skeleton_geometry const sk_cylinder_binded = multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose);
    mesh_cylinder.apply_skinning(sk_cylinder_binded);
    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);
    mesh_cylinder_opengl.draw();

}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    glLineWidth(1.0f);                                                                                 PRINT_OPENGL_ERROR();

}

void scene::setup_shader_skeleton(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniform3f(get_uni_loc(shader_id,"color") , 0.0f,0.0f,0.0f);                                      PRINT_OPENGL_ERROR();

    //size of the lines
    glLineWidth(3.0f);                                                                                 PRINT_OPENGL_ERROR();
}

void scene::draw_skeleton(std::vector<vec3> const& positions) const
{
    // Create temporary a VBO to store data
    GLuint vbo_skeleton=0;
    glGenBuffers(1,&vbo_skeleton);                                                                     PRINT_OPENGL_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER,vbo_skeleton);                                                        PRINT_OPENGL_ERROR();

    // Update data on the GPU
    glBufferData(GL_ARRAY_BUFFER , sizeof(vec3)*positions.size() , &positions[0] , GL_STATIC_DRAW);    PRINT_OPENGL_ERROR();

    // Draw data
    glEnableClientState(GL_VERTEX_ARRAY);                                                              PRINT_OPENGL_ERROR();
    glVertexPointer(3, GL_FLOAT, 0, 0);                                                                PRINT_OPENGL_ERROR();
    glDrawArrays(GL_LINES,0,positions.size());                                                         PRINT_OPENGL_ERROR();

    // Delete temporary VBO
    glDeleteBuffers(1,&vbo_skeleton);                                                                  PRINT_OPENGL_ERROR();
}

scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}


