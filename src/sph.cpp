#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include <GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "render.h"
#include "helpers.h"

using namespace std;
using namespace glm;

vector<vec4> ball_vertices;
vector<uvec3> ball_faces;
vector<vec4> ball_normals;

void sph() {
  line_shader->drawAxis();

  for (int i = 0; i < 5; i++) {
    mat4 T = glm::translate(mat4(1.0), vec3(i * 1.0));

    phong_shader->draw(ball_vertices, ball_faces, ball_normals, T, BLUE);
  }
}

void setupSPH () {
  LoadOBJ("../obj/sphere.obj", ball_vertices, ball_faces, ball_normals);
}

int main (int argc, char* argv[]) {
  initOpenGL();
  setupSPH();

  while (keepLoopingOpenGL()) {
    sph();

    endLoopOpenGL();
  }

  cleanupOpenGL();
}
