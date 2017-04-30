#include "viewer_wrapper.h"
#include "simulation.h"

#include <functional>

#include <igl/viewer/Viewer.h>
#include <igl/jet.h>

#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/window.h>

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
  else if (key == 'S')
    sim->params->show_surface = !sim->params->show_surface;

  return false;
}

nanogui::TextBox *makeSlider(double &property, nanogui::Window *panel) {
  nanogui::TextBox *textbox = new nanogui::TextBox(panel);
  textbox->setFixedSize(Vector2i(60, 25));
  textbox->setValue(to_string(property));

  nanogui::Slider *slider = new nanogui::Slider(textbox);
  slider->setValue(property);
  slider->setFixedWidth(80);
  slider->setCallback([&property, textbox](float value) {
      property = value;
      textbox->setValue(to_string(value));
  });

  return textbox;
}

bool init(igl::viewer::Viewer& viewer, Simulation *sim) {
  Parameters *params = sim->params;

  nanogui::Window *panel = viewer.ngui->addWindow(Vector2i(220, 20),
                                                   "SPH Fluid Simulation");

  // Simulation environment variables.
  viewer.ngui->addGroup("Simulation Parameters");
  viewer.ngui->addVariable("Number of Particles", params->nb_particles);
  viewer.ngui->addVariable("Radius",              params->radius);
  viewer.ngui->addVariable("Mass",                params->mass);
  viewer.ngui->addVariable("Density",             params->density);
  viewer.ngui->addVariable("Viscocity",           params->viscocity);
  viewer.ngui->addVariable("Gravity",             params->gravity);
  viewer.ngui->addVariable("Time Step",           params->time_step);

  viewer.ngui->addWidget("Kernel Size", makeSlider(params->kernel_radius, panel));
  viewer.ngui->addButton("Reset Parameters",
                         [params](){ params->reset(); });

  // Marching cubes stuff.
  viewer.ngui->addGroup("Surface Rendering");
  viewer.ngui->addVariable("Grid Resolution", params->resolution);
  viewer.ngui->addWidget("Offset", makeSlider(params->surface, panel));
  viewer.ngui->addButton("Show surface",
                         [params](){ params->show_surface ^= true; });

  // Simulation controls.
  viewer.ngui->addGroup("Simulation Controls");
  viewer.ngui->addButton("Toggle Simulation",
                         [params](){ params->paused ^= true; });
  viewer.ngui->addButton("Reset Simulation",
                         [sim](){ sim->reset(); });

  // Generate widgets.
  viewer.screen->performLayout();

  viewer.core.show_lines = false;

  return false;
}

bool post_draw(igl::viewer::Viewer& viewer, Simulation *sim) {
  // Take a step.
  if (!sim->params->paused)
    sim->step();

  // Get the current mesh of the simulation.
  MatrixX3d V;
  MatrixX3i F;
  VectorXd C;

  // C will have 0 rows if using marching cubes.
  sim->render(V, F, C);

  MatrixX3d P;
  MatrixX2i E;
  MatrixX3d EC;
  sim->getBounds(P, E, EC);

  // Update the viewer.
  viewer.data.clear();
  viewer.data.set_edges(P, E, EC);
  viewer.data.set_mesh(V, F);

  if (C.rows() > 0) {
    MatrixX3d C_jet;
    igl::jet(C, true, C_jet);

    viewer.data.set_colors(C_jet);
  }

  if (sim->current_time == 0.0)
    viewer.core.align_camera_center(V, F);

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
