
/** TP 5ETI - CPE Lyon - 2015/2016 */

#pragma once

#ifndef SCENE_HPP
#define SCENE_HPP

#include <GL/gl.h>
#include <GL/glew.h>

#include <QTime>

#include "../../lib/3d/mat3.hpp"
#include "../../lib/3d/vec3.hpp"
#include "../../lib/mesh/mesh.hpp"
#include "../../lib/opengl/mesh_opengl.hpp"
#include "../../lib/interface/camera_matrices.hpp"
#include "../../skinning/mesh_skinned.hpp"
#include "../../skinning/skeleton_parent_id.hpp"
#include "../../skinning/skeleton_animation.hpp"

#include <vector>


class myWidgetGL;

class scene
{
public:

    scene();



    /**  Method called only once at the beginning (load off files ...) */
    void load_scene();

    /**  Method called at every frame */
    void draw_scene();

    /** Set the pointer to the parent Widget */
    void set_widget(myWidgetGL* widget_param);


private:

    /** Draw a set of line representing a skeleton */
    void draw_skeleton(std::vector<cpe::vec3> const& positions) const;

    /** Load a texture from a given file and returns its id */
    GLuint load_texture_file(std::string const& filename);

    /** Access to the parent object */
    myWidgetGL* pwidget;

    /** Default id for the texture (white texture) */
    GLuint texture_default;


    /** Ground mesh */
    cpe::mesh mesh_ground;
    /** Ground mesh for OpenGL drawing */
    cpe::mesh_opengl mesh_ground_opengl;

    /** Mesh of the skinned cylinder */
    cpe::mesh_skinned mesh_cylinder;
    /** Mesh of the skinned cylinder for OpenGL drawing */
    cpe::mesh_opengl mesh_cylinder_opengl;

    /** Skeleton of the bind pose for the cylinder */
    cpe::skeleton_geometry sk_cylinder_bind_pose;
    /** Parent id for the skeleton of the cylinder */
    cpe::skeleton_parent_id sk_cylinder_parent_id;
    /** Animation of the skeleton for the cylinder */
    cpe::skeleton_animation sk_cylinder_animation;

    /** Mesh of the skinned monster */
    cpe::mesh_skinned mesh_monster;
    /** Mesh of the skinned monster for OpenGL drawing */
    cpe::mesh_opengl mesh_monster_opengl;
    /** Texture of the monster */
    GLuint texture_monster;

    /** Skeleton of the bind pose for the monster */
    cpe::skeleton_geometry sk_monster_bind_pose;
    /** Parent id for the skeleton of the monster */
    cpe::skeleton_parent_id sk_monster_parent_id;
    /** Animation of the skeleton for the monster */
    cpe::skeleton_animation sk_monster_animation;



    /** The id of the shader to draw meshes */
    GLuint shader_mesh;
    /** The id of the shader to draw skeleton */
    GLuint shader_skeleton;

    QTime time;
    int order = 1;
    int index = 0;
    int next = 1;
    QTime time2;
    int order_monster = 1;
    int index_monster = 0;
    int next_monster = 1;

    void setup_shader_mesh(GLuint shader_id);
    void setup_shader_skeleton(GLuint shader_id);

    cpe::vec3 move = cpe::vec3(0.0, 0.0, 0.0);


};

#endif
