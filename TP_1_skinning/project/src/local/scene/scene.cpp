

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



static cpe::mesh_skinned build_cylinder(float const R,float const H, int const nbT, int const nbH)
{
    mesh_skinned m;

    //Get the increments on the lenght of the tube and on its circumference
    float incT = 2*M_PI / nbT;
    float incH = H / nbH;

    //Create vertice of the cylinder
    for (int k = 0; k < nbH; k++){
        //Update the length of the circle drawn
        float h = k*incH;
        for (int i = 0; i < nbT ; i++){
            vertex_weight_parameter vwp;
            skinning_weight sw, swbis;
            //Update the position on the circle via the angle theta
            float theta = i*incT;
            //Creation of the vertex using its polar coordinates
            m.add_vertex(vec3(R*cos(theta), R*sin(theta), h));


            // //Use strictly 0 or 1 for the weights    //USER
            // if (h>H/2){
            //     sw.joint_id = 1;
            //     sw.weight = 1;
            // } else {
            //     sw.joint_id = 0;
            //     sw.weight = 1;
            // }
            // vwp[0] = sw;

            // Use a linear weight   //USER
            sw.joint_id = 0;
            sw.weight = 1 - h/H;
            swbis.joint_id = 1;
            swbis.weight = h/H;
            vwp[0] = sw;
            vwp[1] = swbis;
            
            //Add the list of weights for the current vertex
            m.add_vertex_weight(vwp);
        }
    }

    //Creates triangles of the cylinder (indexes of the vertices of the triangles)
    for (int i = 0; i < nbH-1; i++){
        for (int j = 0; j < nbT-1; j++){
            //Creation of the rectangle that links two vertices on top of to others
            m.add_triangle_index({i*nbT + j, i*nbT + j+1, (i+1)*nbT + j + 1});
            m.add_triangle_index({i*nbT + j , (i+1)*nbT + j+1, (i+1)*nbT + j});
        }
        //Avoid border effect by linking the last vertices of the circle to the first
        m.add_triangle_index({(i+1)*nbT-1, i*nbT, (i+1)*nbT});
        m.add_triangle_index({(i+1)*nbT-1, (i+1)*nbT, (i+2)*nbT-1});
    }
    //give a specific color to the cylinder
    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}


static void Init_cylinder_skeleton(cpe::skeleton_parent_id &sk_cylinder_parent_id,cpe::skeleton_geometry &sk_cylinder_bind_pose ,float length) //quaternion q
{
    //Attribute a parent to each joint, consider the first as the master joint
    sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);

    //Creation of the joints of the skeleton of the cylinder
    skeleton_joint j00 = skeleton_joint(vec3(0,0,0),cpe::quaternion(0,0,0,1));
    skeleton_joint j11 = skeleton_joint(vec3(0,0,length/2),cpe::quaternion(0,0,0,1));
    skeleton_joint j22 = skeleton_joint(vec3(0,0,length/2),cpe::quaternion(0,0,0,1));

    //Add joints to the skeleton of the bind pose of the cylinder
    sk_cylinder_bind_pose.push_back(j00);
    sk_cylinder_bind_pose.push_back(j11);
    sk_cylinder_bind_pose.push_back(j22);

}

static void Init_monster_skeleton(cpe::skeleton_parent_id &sk_monster_parent_id,cpe::skeleton_geometry &sk_monster_bind_pose)
{
    //Load skeleton organization of the monster
    sk_monster_parent_id.load("data/Monster.skeleton");
    sk_monster_bind_pose.load("data/Monster.skeleton");
}


static void Init_cylinder_animation(cpe::skeleton_parent_id &sk_cylinder_parent_id,cpe::skeleton_geometry sk_cylinder_bind_pose ,cpe::skeleton_animation &sk_cylinder_animation ,float length)
{

    //Creation of the 3 frames of the animation of the cylinder, only the 2nd joint is modified each time
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);     //Initial position
    
    //Rotation of 30° from j0
    skeleton_joint j_30 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_30.orientation.set_axis_angle(vec3(1,0,0),M_PI/6);        
    sk_cylinder_bind_pose[1] = j_30;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);

    //Rotation of 60° from j0
    skeleton_joint j_60 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_60.orientation.set_axis_angle(vec3(1,0,0),M_PI/3);
    sk_cylinder_bind_pose[1] = j_60;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);

    //Rotation of 90° from j0
    skeleton_joint j_90 = skeleton_joint(vec3(0,0,length/2),quaternion(0,0,0,1));
    j_90.orientation.set_axis_angle(vec3(1,0,0),M_PI/2);
    sk_cylinder_bind_pose[1] = j_90;
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);


}

static void Init_monster_animation(cpe::skeleton_parent_id &sk_monster_parent_id,cpe::skeleton_geometry sk_monster_bind_pose ,cpe::skeleton_animation &sk_monster_animation)
{
    //Load the frames of the animations of the monster
    sk_monster_animation.load("data/Monster.animations",sk_monster_bind_pose.size());
}

void scene::load_scene()
{

    float const length = 50.0f;
    float const radius = 4.0f;
    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    texture_monster = load_texture_file("data/Monster.png");
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
    //Init the skeletons
    Init_cylinder_skeleton(sk_cylinder_parent_id, sk_cylinder_bind_pose , length) ;

    //*****************************************//
    // Build monster
    //*****************************************//
    mesh_monster.load("data/Monster.obj");
    //Init the skeleton    
    Init_monster_skeleton(sk_monster_parent_id, sk_monster_bind_pose) ;

    // Modification of the color of the monster depending on the vertex_weights  //USER
    // for (int k = 0; k < mesh_monster.size_vertex_weight() ; k++){
    //     int unique = 0;
    //     if (mesh_monster.vertex_weight(k)[1].joint_id == 0){
    //         unique = 1;
    //     }

    //     vec3 colorToUse = vec3((1.0-unique) * mesh_monster.vertex_weight(k)[0].weight, (1.0-unique) * mesh_monster.vertex_weight(k)[1].weight, unique * mesh_monster.vertex_weight(k)[0].weight);
    //     // std::cout << mesh_monster.vertex_weight(k)[0].joint_id << " " << mesh_monster.vertex_weight(k)[1].joint_id << std::endl;
    //     mesh_monster.add_color(colorToUse);
    // }

    mesh_monster.fill_empty_field_by_default();
    mesh_monster_opengl.fill_vbo(mesh_monster);
    

    
    //Initialize the animations frames of the squeletons
    Init_cylinder_animation(sk_cylinder_parent_id, sk_cylinder_bind_pose, sk_cylinder_animation, length);
    Init_monster_animation(sk_monster_parent_id, sk_monster_bind_pose, sk_monster_animation);



    time.start();
}



void scene::draw_scene()
{
    setup_shader_skeleton(shader_skeleton);

    //Increase the movement of the monster      //USER
    move += order_monster * vec3(1, 0.0, 0.0);
    //Get the frame of the monster to display according to the elapsed time
    if (time.elapsed()>50){
        //Update the index of the monster keyframe
        index_monster = next_monster;
        //Check if there are more keyframe and update the next keyframe
        if ((order_monster > 0) && (next_monster + 1 < sk_monster_animation.size()-1)){
            next_monster +=1;

        }
        //Check if there are no more next keyframe then get the previous keyframe 
        else if ((order > 0) && (next_monster + 1 >= sk_monster_animation.size()-1)){
            order_monster *= -1;
            next_monster -= 1;

        } 
        //Check if there are more previous keyframes
        else if ((order_monster < 0) && (next_monster - 1 >= 0)){
            next_monster -=1;
        } 
        //Check if there are no more previous keyframe then get the next keyframe
        else {
            order_monster *= -1;
            next_monster +=1;
        }
    }
    //Get the frame of the cylinder to display according to the elapsed time
    if (time.elapsed() > 2000){
        //Update the index of the cylinder keyframe
        index = next;
        //Check if there are more keyframe and update the next keyframe
        if ((order > 0) && (next + 1 < sk_cylinder_animation.size())){
            next +=1;
        } 
        //Check if there are no more next keyframe then get the previous keyframe 
        else if ((order > 0) && (next + 1 >= sk_cylinder_animation.size())){
            order *= -1;
            next -= 1;
        } 
        //Check if there are more previous keyframes
        else if ((order < 0) && (next - 1 >= 0)){
            next -=1;
        } 
        //Check if there are no more previous keyframe then get the next keyframe
        else {
            order *= -1;
            next +=1;
        }
        time.restart();

    }

    //Get the interpolation parameter to get a smooth change between keyframes
    float alpha_monster = float(time.elapsed()) / 2000.0;
    float alpha = float(time.elapsed())/50.0;
    //Here we can draw skeletons as 3D segments

    //Interpolation of the frames according to the time     //USER
    skeleton_geometry sk_toUse = interpolated(sk_cylinder_animation[index], sk_cylinder_animation[next], alpha);  // sk_cylinder_animation[index]; //
    skeleton_geometry sk_toUse_monster =  interpolated(sk_monster_animation[index_monster], sk_monster_animation[next_monster], alpha_monster); //sk_monster_animation[index_monster]; //
    
    //Move the monster
    sk_toUse_monster[0].position += move - vec3(75.0, 0.0, 0.0);
    
    //Get the position of the skeleton of the cylinder in global coordinates and draw it
    skeleton_geometry const sk_cylinder_global = local_to_global(sk_toUse,sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
    draw_skeleton(sk_cylinder_bones);        //Draw the skeleton of the cylinder    //USER 

    //Get the position of the skeleton of the monster in global coordinates and draw it   
    skeleton_geometry const sk_monster_global = local_to_global(sk_toUse_monster,sk_monster_parent_id);
    std::vector<vec3> const sk_monster_bones = extract_bones(sk_monster_global,sk_monster_parent_id);
    draw_skeleton(sk_monster_bones);        //Draw the skeleton of the monster  //USER



    //Here we draw the object
    setup_shader_mesh(shader_mesh);
    //Draw the ground
    mesh_ground_opengl.draw();

    //Prepare the skinning
    skeleton_geometry const sk_cylinder_inverse_bind_pose = inversed(sk_cylinder_bind_pose);
    skeleton_geometry const sk_cylinder_binded = multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose);
    //Apply the skinning on the cylinder 
    mesh_cylinder.apply_skinning(sk_cylinder_binded);
    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);
    //Draw the cylinder     //USER
    mesh_cylinder_opengl.draw();

    //Prepare the skinning
    skeleton_geometry const sk_monster_inverse_bind_pose = inversed(local_to_global(sk_monster_bind_pose, sk_monster_parent_id));
    skeleton_geometry const sk_monster_binded = multiply(sk_monster_global,sk_monster_inverse_bind_pose);
    //Apply the skinning on the monster    
    mesh_monster.apply_skinning(sk_monster_binded);
    mesh_monster.fill_normal();
    mesh_monster_opengl.update_vbo_vertex(mesh_monster);
    mesh_monster_opengl.update_vbo_normal(mesh_monster);
    glBindTexture(GL_TEXTURE_2D, texture_monster);
    //Draw the monster          //USER
    mesh_monster_opengl.draw();


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


