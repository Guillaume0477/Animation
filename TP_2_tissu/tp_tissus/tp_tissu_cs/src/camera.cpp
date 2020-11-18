#include "camera.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#define __gl_h_
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

Camera::Camera()
  : pos(3.,-1.5,3.),
  target(0.,0.,0.),
  up(0,1,0),
  distance(glm::distance(pos, target)),
  buttons_mouse(0),
  screen_width(800),
  screen_height(800)
{}


int Camera::height() const {return screen_height;}
int Camera::width() const {return screen_width;}

glm::mat4 Camera::projection() const
{
  return glm::perspective(glm::radians(45.0f), (float)screen_width / (float)screen_height, 0.1f, 100.0f);
}

glm::mat4 Camera::view() const
{
  const glm::vec3 new_pos(cos(heading)*cos(pitch),sin(pitch), -sin(heading)*cos(pitch));
  return glm::lookAt(
      glm::normalize((new_pos * pos)-target) * distance + target,
      target,
      up); 
}

void Camera::common_reshape(int w, int h)
{
  screen_width  = w;
  screen_height = h;
}
void Camera::common_mouse (int button, int state, int x, int y)
{
  origin_mouse = glm::vec2(x, y);
  if (state == GLUT_DOWN)
    buttons_mouse = button;
  else
    button = 0; 
}

void Camera::common_motion(int x, int y)
{

  if (buttons_mouse == GLUT_LEFT_BUTTON)
  { 
    pitch -= float(origin_mouse.y - y) / (float)screen_height * 5.;
    heading -= float(origin_mouse.x - x) / (float)screen_width * 5.;

    pitch = pitch > 1.5 ? 1.5 : pitch < -1.5 ? -1.5 : pitch;
    heading = glm::mod(heading + 3.14f, 6.24f) - 3.14;
  }
  else if (buttons_mouse == GLUT_RIGHT_BUTTON)
  {
    distance += float (origin_mouse.y - y) * 0.2;
    distance = distance < 1. ? 1. : distance > 20. ? 20. : distance;
  }

  origin_mouse = glm::vec2(x, y);
}

glm::vec3 Camera::position() const
{
  const glm::vec3 new_pos(cos(heading)*cos(pitch),sin(pitch), -sin(heading)*cos(pitch));
  return glm::normalize((new_pos * pos)-target) * distance + target;
}

glm::vec3 Camera::lookat() const
{
  return target;
}
