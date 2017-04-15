#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <igl/readOBJ.h>
#include <igl/viewer/Viewer.h>

using namespace std;
using namespace Eigen;

MatrixX3d V;
MatrixX3i F;

bool time_paused = false;

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier) {
  if (key == ' ')
    time_paused = !time_paused;

  return false;
}

bool next_frame(igl::viewer::Viewer& viewer) {
  if (!time_paused)
    return false;

  viewer.data.set_mesh(V, F);

  for (int i = 0; i < V.rows(); i++) {
    V(i, 0) += 0.01;
  }

  // Signal to render.
  glfwPostEmptyEvent();

  return false;
}

int main (int argc, char* argv[]) {
  igl::readOBJ("../obj/sphere.obj", V, F);

  igl::viewer::Viewer viewer;
  viewer.callback_key_down = &key_down;
  viewer.callback_post_draw = &next_frame;

  viewer.data.set_mesh(V, F);

  viewer.launch();
}
