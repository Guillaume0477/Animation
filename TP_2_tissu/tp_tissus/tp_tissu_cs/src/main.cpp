#include <chrono>
#include <iostream>
#include <fstream>
#include <ratio>
#include <string>

#define GLEW_STATIC 1
#include <GL/glew.h>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#define __gl_h_
#include <GLUT/glut.h>
#define MACOSX_COMPATIBILITY GLUT_3_2_CORE_PROFILE
#else
#include <GL/glut.h>
#define MACOSX_COMPATIBILITY 0
#endif
#include "glhelper.h"
#include "camera.h"

const int N = 30;
const int dt = 10; //ms

GLuint VAO;
GLuint buffers[4];
int n_elements;

// camera
Camera cam;

// Current program id
GLuint program_id;
GLuint program_integration_id;
GLuint program_force_id;

int nframe;
std::chrono::time_point<std::chrono::high_resolution_clock> start;

void init()
{
  start = std::chrono::high_resolution_clock::now();

  program_id = glhelper::create_program_from_file("shaders/basic.vs", "shaders/basic.fs");
  glhelper::load_texture("cloth.png");
  program_integration_id = glhelper::create_program_from_file("shaders/integration_step.cs");
  glUseProgram(program_integration_id);
  glUniform1ui(glGetUniformLocation(program_integration_id, "N"),N);
  glUniform1f(glGetUniformLocation(program_integration_id, "dt"),dt/100.f);

  program_force_id = glhelper::create_program_from_file("shaders/force.cs");
  glUseProgram(program_force_id);
  glUniform1ui(glGetUniformLocation(program_force_id, "N"),N);
  glUniform1f(glGetUniformLocation(program_force_id, "dt"),dt/100.f);

  std::vector<glm::vec4> positions(N*N);
  std::vector<glm::vec2> uvs(N*N);
  std::vector<glm::vec4> vitesses(N*N);
  std::vector<glm::vec4> forces(N*N);
  std::vector<unsigned> indices;
  indices.reserve(2*N*(N-1));

  float delta = 1./(float(N)-1.);
  for(auto i = 0u; i < N; ++i)
    for(auto j = 0u; j < N; ++j)
  {
    positions[j+i*N]= glm::vec4(-1.f+2.f*j*delta, 0.f, -1.f+2.f*i*delta, 0.f);
    uvs[j+i*N]= glm::vec2(i*delta, j*delta);
    vitesses[j+i*N] = glm::vec4(0.f,0.f,0.f, 0.f);
    forces[j+i*N] = glm::vec4(0.f,0.f,0.f, 0.f);

  }
  for(int i = 0; i < N*(N-1); ++i)
  {
    if( (i+1)%N != 0 )
    {
      indices.push_back(i);
      indices.push_back(i+1);
      indices.push_back(i+N);

      indices.push_back(i+1);
      indices.push_back(i+N+1);
      indices.push_back(i+N);
    }
  }
  n_elements = indices.size();

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  GLuint ebo;
  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

  glGenBuffers(4, buffers);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*positions.size(), positions.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*vitesses.size(),vitesses.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*forces.size(),forces.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(2);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[3]);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2)*uvs.size(),uvs.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(3);

  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glEnable(GL_DEPTH_TEST);
}

void compute_fps()
{
  ++nframe;
  if(nframe == 100)
  {
    auto stop = std::chrono::high_resolution_clock::now();
    auto diff = (stop-start).count();
    start = stop;
    std::cout << 100./(float(diff) / 1e9) << std::endl;
    nframe = 0;
  }
}

void set_uniform_mvp(GLuint program)
{
  GLint mvp_id = glGetUniformLocation(program, "MVP");
  if (mvp_id != -1)
  {
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 mvp = cam.projection()*cam.view()*model;
    glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &mvp[0][0]);
  }
}

static void display_callback()
{
  glUseProgram(program_force_id); CHECK_GL_ERROR();
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffers[0]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, buffers[1]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, buffers[2]);

  glDispatchCompute(N, N, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  glUseProgram(program_integration_id); CHECK_GL_ERROR();
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffers[0]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, buffers[1]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, buffers[2]);

  glDispatchCompute(N, N, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(2);

  glBindBuffer(GL_ARRAY_BUFFER, buffers[3]);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(3);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glUseProgram(program_id); CHECK_GL_ERROR();
  set_uniform_mvp(program_id);

  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);

  glutSwapBuffers();

  compute_fps();
}

static void keyboard_callback(unsigned char key, int, int)
{
  switch (key)
  {
    case 'p':
      glhelper::print_screen(cam.width(), cam.height());
      break;
    case 'q':
    case 'Q':
    case 27:
      exit(0);
  }
}

static void reshape_callback(int width, int height)
{
  cam.common_reshape(width,height);

  glViewport(0,0, width, height);
}
static void mouse_callback (int button, int action, int x, int y)
{
  cam.common_mouse(button, action, x, y);
}

static void motion_callback(int x, int y)
{
  cam.common_motion(x, y);
}

static void timer_callback(int)
{
  glutTimerFunc(dt, timer_callback, 0);
  glutPostRedisplay();
}

// From : https://zestedesavoir.com/tutoriels/1554/introduction-aux-compute-shaders/
void printWorkGroupsCapabilities() {
  int workgroup_count[3];
  int workgroup_size[3];
  int workgroup_invocations;

  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &workgroup_count[0]);
  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &workgroup_count[1]);
  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &workgroup_count[2]);

  printf ("Taille maximale des workgroups:\n\tx:%u\n\ty:%u\n\tz:%u\n",
      workgroup_count[0], workgroup_count[1], workgroup_count[2]);

  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &workgroup_size[0]);
  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &workgroup_size[1]);
  glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &workgroup_size[2]);

  printf ("Nombre maximal d'invocation locale:\n\tx:%u\n\ty:%u\n\tz:%u\n",
      workgroup_size[0], workgroup_size[1], workgroup_size[2]);

  glGetIntegerv (GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &workgroup_invocations);
  printf ("Nombre maximum d'invocation de workgroups:\n\t%u\n", workgroup_invocations);

  GLint ssbo_size;
  glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &ssbo_size);
  std::cout << "GL_MAX_SHADER_STORAGE_BLOCK_SIZE is " << ssbo_size/std::mega::num << " Mbytes." << std::endl;
}

int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | MACOSX_COMPATIBILITY);
  glutInitWindowSize(cam.height(), cam.width());
  glutCreateWindow("opengl");
  glutDisplayFunc(display_callback);
  glutMotionFunc(motion_callback);
  glutMouseFunc(mouse_callback);
  glutKeyboardFunc(keyboard_callback);
  glutReshapeFunc(reshape_callback);
  glutTimerFunc(dt, timer_callback, 0);

  glewExperimental=true;
  glewInit();

  printWorkGroupsCapabilities();
  init();
  glutMainLoop();

  return 0;
}
