#ifndef RENDER_H
#define RENDER_H

#include <vector>
#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

struct Shader {
  int vaoIndex;

  GLuint programId;

  GLint projection_matrix_location;
  GLint model_matrix_location;
  GLint view_matrix_location;

  const glm::mat4& view;
  const glm::mat4& proj;

  GLint light_position_location;

  Shader (glm::mat4* view_p, glm::mat4* proj_p) :
    view(*view_p), proj(*proj_p) { }

  virtual void setup() = 0;
  virtual ~Shader() { };
};

struct PhongShader: public Shader {
  GLint obj_color_location;
  GLint eye_location;

  PhongShader (glm::mat4* view_p, glm::mat4* proj_p) :
    Shader(view_p, proj_p) {
  }

  virtual void setup();
  void draw (const std::vector<glm::vec4>& vertices,
             const std::vector<glm::uvec3>& faces,
             const std::vector<glm::vec4>& normals,
             const glm::mat4& model, const glm::vec4& color);
  virtual ~PhongShader() { };
};

struct LineShader: public Shader {
  GLint line_color_location = 0;

  LineShader (glm::mat4* view_p, glm::mat4* proj_p) :
    Shader(view_p, proj_p) {
  }

  virtual void setup();
  void draw (const std::vector<glm::vec4>& vertices,
             const std::vector<glm::uvec2>& segments,
             const glm::mat4& model_matrix, const glm::vec4& color);
  void drawLineSegment (const glm::vec3& u, const glm::vec3& v,
                        const glm::vec4& color);
  void drawAxis();
  virtual ~LineShader() { };
};

extern glm::vec4 LIGHT_POSITION;

extern const float kNear;
extern const float kFar;
extern const float kFov;

extern float camera_distance;

extern glm::vec3 up;
extern glm::vec3 look;
extern glm::vec3 tangent;
extern glm::mat3 orientation;

extern glm::vec3 eye;
extern glm::vec3 center;

extern glm::mat4 view_matrix;
extern glm::mat4 projection_matrix;

extern const std::string window_title;

extern const int WIDTH;
extern const int HEIGHT;

extern bool timePaused;
extern bool showFloor;
extern bool do_action;

extern bool click_enabled;
extern glm::vec3 click_position;

extern PhongShader *phong_shader;
extern LineShader *line_shader;

const std::string LINE_VERT = "../src/shaders/basic.vert";
const std::string LINE_GEOM = "../src/shaders/line.geom";
const std::string LINE_FRAG = "../src/shaders/line.frag";

const std::string PHONG_VERT = "../src/shaders/phong.vert";
const std::string PHONG_GEOM = "../src/shaders/phong.geom";
const std::string PHONG_FRAG = "../src/shaders/phong.frag";

enum {
  kVertexBuffer,
  kIndexBuffer,
  kVertexNormalBuffer,
  kNumVbos
};

enum {
  kLineSegVao,
  kPhongVao,
  kNumVaos
};

extern GLuint array_objects[kNumVaos];
extern GLuint buffer_objects[kNumVaos][kNumVbos];

void initOpenGL();
bool keepLoopingOpenGL();
void cleanupOpenGL();
void endLoopOpenGL();

GLuint setupShader (const char* shaderName, GLenum shaderType);

extern const char* OpenGlErrorToString(GLenum error);

#define CHECK_SUCCESS(x) \
  if (!(x)) {            \
    glfwTerminate();     \
    exit(EXIT_FAILURE);  \
  }

#define CHECK_GL_SHADER_ERROR(id)                                           \
  {                                                                         \
    GLint status = 0;                                                       \
    GLint length = 0;                                                       \
    glGetShaderiv(id, GL_COMPILE_STATUS, &status);                          \
    glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);                         \
    if (!status) {                                                          \
      string log(length, 0);                                                \
      glGetShaderInfoLog(id, length, nullptr, &log[0]);                     \
      cerr << "Line :" << __LINE__ << " OpenGL Shader Error: Log = \n"      \
                << &log[0];                                                 \
      glfwTerminate();                                                      \
      exit(EXIT_FAILURE);                                                   \
    }                                                                       \
  }

#define CHECK_GL_PROGRAM_ERROR(id)                                          \
  {                                                                         \
    GLint status = 0;                                                       \
    GLint length = 0;                                                       \
    glGetProgramiv(id, GL_LINK_STATUS, &status);                            \
    glGetProgramiv(id, GL_INFO_LOG_LENGTH, &length);                        \
    if (!status) {                                                          \
      string log(length, 0);                                                \
      glGetProgramInfoLog(id, length, nullptr, &log[0]);                    \
      cerr << "Line :" << __LINE__ << " OpenGL Program Error: Log = \n"     \
                << &log[0];                                                 \
      glfwTerminate();                                                      \
      exit(EXIT_FAILURE);                                                   \
    }                                                                       \
  }

#define CHECK_GL_ERROR(statement)                                           \
  {                                                                         \
    { statement; }                                                          \
    GLenum error = GL_NO_ERROR;                                             \
    if ((error = glGetError()) != GL_NO_ERROR) {                            \
      cerr << "Line :" << __LINE__ << " OpenGL Error: code  = " << error    \
                << " description =  " << OpenGlErrorToString(error);        \
      glfwTerminate();                                                      \
      exit(EXIT_FAILURE);                                                   \
    }                                                                       \
  }

#endif
