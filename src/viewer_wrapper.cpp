#include "viewer_wrapper.h"
#include "simulation.h"

#include <functional>

#include <igl/viewer/Viewer.h>
#include <igl/jet.h>

using namespace std;
using namespace Eigen;

// anonymous namespace to handle LibIGL viewer.
namespace {

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int mod,
              Simulation *sim) {
  if (key == ' ')
    sim->params->paused = !sim->params->paused;
  else if (key == 'R')
    sim->reset();

  return false;
}

bool init(igl::viewer::Viewer& viewer, Simulation *sim) {
  viewer.ngui->addWindow(Vector2i(220, 10), "SPH Fluid Simulation");

  // Simulation environment variables.
  viewer.ngui->addGroup("Simulation Parameters");
  viewer.ngui->addVariable("Number of Particles", sim->params->nb_particles);
  viewer.ngui->addVariable("Radius",              sim->params->radius);
  viewer.ngui->addVariable("Mass",                sim->params->mass);
  viewer.ngui->addVariable("Density",             sim->params->density);
  viewer.ngui->addVariable("Viscocity",           sim->params->viscocity);
  viewer.ngui->addVariable("Gravity",             sim->params->gravity);
  viewer.ngui->addVariable("Time Step",           sim->params->time_step);
  viewer.ngui->addVariable("Current Time",        sim->current_time, false);
  viewer.ngui->addButton("Reset Parameters",
                         [sim](){ sim->params->reset(); });

  // Simulation controls..
  viewer.ngui->addGroup("Simulation Controls");
  viewer.ngui->addButton("Toggle Simulation",
                         [sim](){ sim->params->paused = !sim->params->paused; });
  viewer.ngui->addButton("Reset Simulation",
                         [sim](){ sim->reset(); });

  // Generate widgets.
  viewer.screen->performLayout();

  return false;
}

bool post_draw(igl::viewer::Viewer& viewer, Simulation *sim) {
  // Take a step.
  if (!sim->params->paused)
    sim->step();

  // Get the current mesh of the simulation.
  MatrixX3d V;
  MatrixX3i F;
  VectorXd V_rho;
  sim->render(V, F, V_rho);

  MatrixX3d VC;
  igl::jet(V_rho, true, VC);

  MatrixX3d P;
  MatrixX2i E;
  MatrixX3d EC;
  sim->getBounds(P, E, EC);

  // Update the viewer.
  viewer.data.clear();
  viewer.data.set_mesh(V, F);
  viewer.data.set_colors(VC);
  viewer.data.set_edges(P, E, EC);

  // Signal to render.
  glfwPostEmptyEvent();

  return false;
}

} // end anonymous namespace to handle LibIGL viewer.

void ViewerWrapper::start() {
  igl::viewer::Viewer viewer;
  viewer.callback_key_down = bind(key_down,
                                  placeholders::_1,
                                  placeholders::_2,
                                  placeholders::_3,
                                  sim_);
  viewer.callback_init = bind(init, placeholders::_1, sim_);
  viewer.callback_post_draw = bind(post_draw, placeholders::_1, sim_);

  viewer.launch();
}
