#include "render.h"
#include "helpers.h"

#include <vector>
#include <string>
#include <iostream>

#include <GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace std;
using namespace glm;

enum {
  kMouseModeCamera,
  kClickMode,
  kNumMouseModes
};

GLuint array_objects[kNumVaos];
GLuint buffer_objects[kNumVaos][kNumVbos];

int current_mouse_mode = 0;

vec4 LIGHT_POSITION = vec4(10.0f, 10.0f, 10.0f, 1.0f);

const float kNear = 0.0001f;
const float kFar = 150.0f;
const float kFov = 45.0f;
float camera_distance = 2.0f;

vec3 eye = vec3(5.0f, 5.0f, 5.0f);
vec3 center = vec3(0.0f, 0.0f, 0.0f);

vec3 up = vec3(0.0f, 1.0f, 0.0f);
vec3 look = eye - center;
vec3 tangent = cross(up, look);
mat3 orient = mat3(tangent, up, look);

mat4 view_matrix;
mat4 projection_matrix;

const std::string window_title = "SPH Fluid Simulation";

const int WIDTH = 800;
const int HEIGHT = 600;
int window_width = WIDTH;
int window_height = HEIGHT;

float last_x = 0.0f, last_y = 0.0f, current_x = 0.0f, current_y = 0.0f;
bool drag_state = false;
bool button_press = false;
int current_button = -1;
const float pan_speed = 0.1f;
const float rotation_speed = 0.05f;
const float zoom_speed = 0.05f;

bool timePaused = true;
bool showWire = false;

bool do_action = false;
size_t action_counter = 0;
size_t prev_action_counter = 0;

bool click_enabled = false;
vec3 click_position;

GLFWwindow* window;

PhongShader* phong_shader;
LineShader* line_shader;

const char* OpenGlErrorToString(GLenum error) {
  switch (error) {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
      break;
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
      break;
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
      break;
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
      break;
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
      break;
    default:
      return "Unknown Error";
      break;
  }
  return "";
}

GLuint setupShader (const char* shaderName, GLenum shaderType) {
  GLuint shaderId = 0;
  CHECK_GL_ERROR(shaderId = glCreateShader(shaderType));
  CHECK_GL_ERROR(glShaderSource(shaderId, 1, &shaderName, nullptr));
  glCompileShader(shaderId);
  CHECK_GL_SHADER_ERROR(shaderId);
  return shaderId;
}

void errorCallback (int error, const char* description) {
  cerr << "GLFW Error: " << description << "\n";
}

void KeyCallback (GLFWwindow* window, int key, int scancode, int action, int mods) {
  do_action = false;

  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GL_TRUE);
  else if (action == GLFW_PRESS && key == GLFW_KEY_SPACE) {
    do_action = true;
  }
  else if (action != GLFW_RELEASE) {
    if (key == GLFW_KEY_W)
      camera_distance = fmax(0.1f, camera_distance - zoom_speed);
    else if (key == GLFW_KEY_S)
      camera_distance += zoom_speed;
    else if (key == GLFW_KEY_A)
      center -= pan_speed * tangent;
    else if (key == GLFW_KEY_D)
      center += pan_speed * tangent;
    else if (key == GLFW_KEY_M)
      showWire = !showWire;
    else if (key == GLFW_KEY_T)
      timePaused = !timePaused;
    else if (key == GLFW_KEY_P) {
      current_mouse_mode = (current_mouse_mode + 1) % kNumMouseModes;
      click_enabled = (current_mouse_mode == kClickMode);
    }
  }
}

vec3 mouseWorldPosition(GLFWwindow* window) {
  vec4 viewport = vec4(0.0f, 0.0f, WIDTH, HEIGHT);

  double x, y;
  float z;

  glfwGetCursorPos(window, &x, &y);
  glReadPixels(x, viewport[3]-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

  vec3 mouseNDC = vec3(x, viewport[3]-y, z);

  return unProject(mouseNDC, view_matrix, projection_matrix, viewport);
}

void cursorPosCallback(GLFWwindow* window, double mouse_x, double mouse_y) {
  last_x = current_x;
  last_y = current_y;
  current_x = mouse_x;
  current_y = window_height - mouse_y;

  float delta_x = current_x - last_x;
  float delta_y = current_y - last_y;

  if (delta_x * delta_x + delta_y * delta_y < 1e-5)
    return;

  vec3 mouse_direction = normalize(vec3(delta_x, delta_y, 0.0f));
  vec3 mouse = vec3(mouse_direction.y, -mouse_direction.x, 0.0f);
  vec3 axis = normalize(orient * mouse);

  if (drag_state && current_button == GLFW_MOUSE_BUTTON_LEFT) {
    if (current_mouse_mode == kMouseModeCamera) {
      up = vec3(0.0f, 1.0f, 0.0f);
      orient = mat3(rotate(rotation_speed, axis) * mat4(orient));
      look = column(orient, 2);
      tangent = cross(up, look);
    }
  }
  if (current_button == GLFW_MOUSE_BUTTON_LEFT && current_mouse_mode == kClickMode) {
    click_position = mouseWorldPosition(window);
  }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  drag_state = (action == GLFW_PRESS);
  current_button = button;
}

void initOpenGL() {
  if (!glfwInit())
    exit(EXIT_FAILURE);

  glfwSetErrorCallback(errorCallback);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  window = glfwCreateWindow(window_width, window_height,
                            &window_title[0], nullptr, nullptr);

  CHECK_SUCCESS(window != nullptr);

  glfwMakeContextCurrent(window);
  CHECK_SUCCESS(glewInit() == GLEW_OK);
  glGetError();  // clear GLEW's error for it

  glfwSetKeyCallback(window, KeyCallback);
  glfwSetCursorPosCallback(window, cursorPosCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSwapInterval(1);

  cout << "Renderer: " << glGetString(GL_RENDERER) << "\n";
  cout << "OpenGL version supported:" << glGetString(GL_VERSION) << "\n";

  // Setup our VAOs.
  CHECK_GL_ERROR(glGenVertexArrays(kNumVaos, array_objects));

  // Generate buffer objects
  for (int i = 0; i < kNumVaos; ++i)
    CHECK_GL_ERROR(glGenBuffers(kNumVbos, &buffer_objects[i][0]));

  // glEnable(GL_CULL_FACE);
  // glCullFace(GL_BACK);

  glClearColor(0.9f, 0.9f, 0.9f, 0.9f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glDepthFunc(GL_LESS);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  phong_shader = new PhongShader(&view_matrix, &projection_matrix);
  phong_shader->setup();
  line_shader = new LineShader(&view_matrix, &projection_matrix);
  line_shader->setup();
}

bool keepLoopingOpenGL() {
  if (glfwWindowShouldClose(window))
    return false;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glfwGetFramebufferSize(window, &window_width, &window_height);
  glViewport(0, 0, window_width, window_height);

  // Compute our view, and projection matrices.
  eye = center + camera_distance * look;

  view_matrix = lookAt(eye, center, up);
  projection_matrix = perspective(static_cast<float>(kFov * (M_PI / 180.0f)),
                                  static_cast<float>(window_width) / window_height,
                                  kNear, kFar);

  return true;
}

void endLoopOpenGL() {
  glfwPollEvents();
  glfwSwapBuffers(window);
}

void cleanupOpenGL() {
  delete phong_shader;
  delete line_shader;

  glfwDestroyWindow(window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}

void PhongShader::setup() {
  this->vaoIndex = kPhongVao;

  // shaders
  string vertex_shader = loadShader(PHONG_VERT);
  GLuint vertex_shader_id = setupShader(vertex_shader.c_str(), GL_VERTEX_SHADER);

  string geometry_shader = loadShader(PHONG_GEOM);
  GLuint geometry_shader_id = setupShader(geometry_shader.c_str(), GL_GEOMETRY_SHADER);

  string fragment_shader = loadShader(PHONG_FRAG);
  GLuint fragment_shader_id = setupShader(fragment_shader.c_str(), GL_FRAGMENT_SHADER);

  // program
  GLuint& program_id = this->programId;
  CHECK_GL_ERROR(program_id = glCreateProgram());
  CHECK_GL_ERROR(glAttachShader(program_id, vertex_shader_id));
  CHECK_GL_ERROR(glAttachShader(program_id, geometry_shader_id));
  CHECK_GL_ERROR(glAttachShader(program_id, fragment_shader_id));

  // attributes
  CHECK_GL_ERROR(glBindAttribLocation(program_id, 0, "vertex_position"));
  CHECK_GL_ERROR(glBindAttribLocation(program_id, 1, "vertex_normal"));
  CHECK_GL_ERROR(glBindFragDataLocation(program_id, 0, "fragment_color"));

  glLinkProgram(program_id);
  CHECK_GL_PROGRAM_ERROR(program_id);

  // uniforms
  GLint& projection_matrix_location = this->projection_matrix_location;
  CHECK_GL_ERROR(projection_matrix_location = glGetUniformLocation(program_id, "projection"));
  GLint& model_matrix_location = this->model_matrix_location;
  CHECK_GL_ERROR(model_matrix_location = glGetUniformLocation(program_id, "model"));
  GLint& view_matrix_location = this->view_matrix_location;
  CHECK_GL_ERROR(view_matrix_location = glGetUniformLocation(program_id, "view"));

  GLint& light_position_location = this->light_position_location;
  CHECK_GL_ERROR(light_position_location = glGetUniformLocation(program_id, "light_position"));

  GLint& obj_color_location = this->obj_color_location;
  CHECK_GL_ERROR(obj_color_location = glGetUniformLocation(program_id, "obj_color"));

  GLint& eye_location = this->eye_location;
  CHECK_GL_ERROR(eye_location = glGetUniformLocation(program_id, "eye"));
}

void PhongShader::draw(const vector<vec4>& vertices,
                       const vector<uvec3>& faces,
                       const vector<vec4>& normals,
                       const mat4& model, const vec4& color) {
  CHECK_GL_ERROR(glUseProgram(this->programId));

  CHECK_GL_ERROR(glUniformMatrix4fv(this->model_matrix_location, 1, GL_FALSE,
                                    &model[0][0]));
  CHECK_GL_ERROR(glUniformMatrix4fv(this->view_matrix_location, 1, GL_FALSE,
                                    &this->view[0][0]));
  CHECK_GL_ERROR(glUniformMatrix4fv(this->projection_matrix_location, 1,
                                    GL_FALSE, &this->proj[0][0]));

  CHECK_GL_ERROR(glUniform4fv(this->light_position_location, 1, &LIGHT_POSITION[0]));

  CHECK_GL_ERROR(glUniform4fv(this->obj_color_location, 1, &color[0]));

  CHECK_GL_ERROR(glUniform4fv(this->eye_location, 1, &eye[0]));

  CHECK_GL_ERROR(glBindVertexArray(array_objects[this->vaoIndex]));

  CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER,
                              buffer_objects[this->vaoIndex][kVertexBuffer]));
  CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                              sizeof(float) * vertices.size() * 4,
                              &vertices[0], GL_STATIC_DRAW));

  CHECK_GL_ERROR(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0));
  CHECK_GL_ERROR(glEnableVertexAttribArray(0));

  CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER,
                              buffer_objects[this->vaoIndex][kVertexNormalBuffer]));
  CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                              sizeof(float) * normals.size() * 4,
                              &normals[0], GL_STATIC_DRAW));

  CHECK_GL_ERROR(glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0));
  CHECK_GL_ERROR(glEnableVertexAttribArray(1));

  CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                              buffer_objects[this->vaoIndex][kIndexBuffer]));
  CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                              sizeof(uint32_t) * faces.size() * 3,
                              &faces[0], GL_STATIC_DRAW));

  CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES, faces.size() * 3,
                                GL_UNSIGNED_INT, 0));
}

void LineShader::setup() {
  this->vaoIndex = kLineSegVao;

  // shaders
  string vertex_shader = loadShader(LINE_VERT).c_str();
  GLuint vertex_shader_id = setupShader(vertex_shader.c_str(),
                                        GL_VERTEX_SHADER);

  string geometry_shader = loadShader(LINE_GEOM).c_str();
  GLuint geometry_shader_id = setupShader(geometry_shader.c_str(),
                                          GL_GEOMETRY_SHADER);

  string fragment_shader = loadShader(LINE_FRAG).c_str();
  GLuint fragment_shader_id = setupShader(fragment_shader.c_str(),
                                          GL_FRAGMENT_SHADER);

  // program
  GLuint& program_id = this->programId;
  CHECK_GL_ERROR(program_id = glCreateProgram());
  CHECK_GL_ERROR(glAttachShader(program_id, vertex_shader_id));
  CHECK_GL_ERROR(glAttachShader(program_id, geometry_shader_id));
  CHECK_GL_ERROR(glAttachShader(program_id, fragment_shader_id));

  // attributes
  CHECK_GL_ERROR(glBindAttribLocation(program_id, 0, "vertex_position"));
  CHECK_GL_ERROR(glBindFragDataLocation(program_id, 0, "fragment_color"));

  glLinkProgram(program_id);
  CHECK_GL_PROGRAM_ERROR(program_id);

  // uniforms
  GLint& model_matrix_location = this->model_matrix_location;
  CHECK_GL_ERROR(model_matrix_location = glGetUniformLocation(program_id, "model"));
  GLint& view_matrix_location = this->view_matrix_location;
  CHECK_GL_ERROR(view_matrix_location = glGetUniformLocation(program_id, "view"));
  GLint& projection_matrix_location = this->projection_matrix_location;
  CHECK_GL_ERROR(projection_matrix_location = glGetUniformLocation(program_id, "projection"));

  GLint& line_color_location = this->line_color_location;
  CHECK_GL_ERROR(line_color_location = glGetUniformLocation(program_id, "line_color"));
}

void LineShader::draw(const vector<vec4>& vertices,
                      const vector<uvec2>& segments,
                      const mat4& model_matrix,
                      const vec4& color) {
  CHECK_GL_ERROR(glUseProgram(this->programId));

  CHECK_GL_ERROR(glUniformMatrix4fv(this->model_matrix_location, 1, GL_FALSE,
                                    &model_matrix[0][0]));
  CHECK_GL_ERROR(glUniformMatrix4fv(this->view_matrix_location, 1, GL_FALSE,
                                    &this->view[0][0]));
  CHECK_GL_ERROR(glUniformMatrix4fv(this->projection_matrix_location, 1, GL_FALSE,
                                    &this->proj[0][0]));

  CHECK_GL_ERROR(glUniform4fv(this->line_color_location, 1, &color[0]));

  CHECK_GL_ERROR(glBindVertexArray(array_objects[this->vaoIndex]));

  CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER,
                              buffer_objects[this->vaoIndex][kVertexBuffer]));
  CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                              sizeof(float) * vertices.size() * 4,
                              &vertices[0], GL_STATIC_DRAW));

  CHECK_GL_ERROR(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0));
  CHECK_GL_ERROR(glEnableVertexAttribArray(0));

  CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                              buffer_objects[this->vaoIndex][kIndexBuffer]));
  CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                              sizeof(uint32_t) * segments.size() * 2,
                              &segments[0], GL_STATIC_DRAW));

  CHECK_GL_ERROR(glDrawElements(GL_LINE_STRIP, segments.size() * 2,
                                GL_UNSIGNED_INT, 0));
}

void LineShader::drawAxis () {
  vector<vec4> x_axis_vertices;
  vector<uvec2> x_axis_segments;

  vec4 point1 = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  vec4 point2 = vec4(1.0f, 0.0f, 0.0f, 1.0f);

  x_axis_vertices.push_back(point1);
  x_axis_vertices.push_back(point2);
  x_axis_segments.push_back(uvec2(1, 0));

  this->draw(x_axis_vertices, x_axis_segments, mat4(), GREEN);

  vector<vec4> z_axis_vertices;
  vector<uvec2> z_axis_segments;

  vec4 point3 = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  vec4 point4 = vec4(0.0f, 0.0f, 1.0f, 1.0f);

  z_axis_vertices.push_back(point3);
  z_axis_vertices.push_back(point4);
  z_axis_segments.push_back(uvec2(1, 0));

  this->draw(z_axis_vertices, z_axis_segments, mat4(), BLUE);

  vector<vec4> y_axis_vertices;
  vector<uvec2> y_axis_segments;

  vec4 point5 = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  vec4 point6 = vec4(0.0f, 1.0f, 0.0f, 1.0f);

  y_axis_vertices.push_back(point5);
  y_axis_vertices.push_back(point6);
  y_axis_segments.push_back(uvec2(1, 0));

  this->draw(y_axis_vertices, y_axis_segments, mat4(), RED);
}

void LineShader::drawLineSegment(const vec3& u, const vec3& v,
                                 const vec4& color) {
  vector<vec4> points;
  points.push_back(vec4(u, 1.0));
  points.push_back(vec4(v, 1.0));

  vector<uvec2> edges;
  edges.push_back(uvec2(0, 1));

  this->draw(points, edges, mat4(), color);
}
