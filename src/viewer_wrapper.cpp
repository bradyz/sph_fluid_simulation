#include "viewer_wrapper.h"
#include "simulation.h"
#include "timer.h"

#include <igl/jet.h>
#include <igl/viewer/Viewer.h>

#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/window.h>

using namespace std;
using namespace Eigen;

// anonymous namespace for LibIGL helpers and callbacks.
namespace {

// Slider and textbox combo.
nanogui::TextBox *makeSlider(double &property, nanogui::Window *panel) {
  nanogui::TextBox *textbox = new nanogui::TextBox(panel);
  textbox->setFontSize(16);
  textbox->setValue(to_string(property));

  nanogui::Slider *slider = new nanogui::Slider(textbox);
  slider->setValue(property);

  // Get the property by reference so it updates in parameters.
  slider->setCallback([&property, textbox](float value) {
      property = value;
      textbox->setValue(to_string(value));
  });

  return textbox;
}

bool init(igl::viewer::Viewer& viewer, ViewerWrapper *wrapper) {
  Simulation *sim = wrapper->sim;
  Parameters *params = wrapper->sim->params;

  nanogui::Window *panel = viewer.ngui->addWindow(Vector2i(220, 20),
                                                  "SPH Fluid Simulation");

  //////////////////////////////////////////////////////////////////////
  // Simulation Parameters.
  //////////////////////////////////////////////////////////////////////
  viewer.ngui->addGroup("Simulation Parameters");
  viewer.ngui->addVariable("Number of Particles", params->nb_particles);
  viewer.ngui->addVariable("Radius",              params->radius);
  viewer.ngui->addVariable("Mass",                params->mass);
  viewer.ngui->addVariable("Density",             params->density);
  viewer.ngui->addVariable("Viscocity",           params->viscocity);
  viewer.ngui->addVariable("Gravity",             params->gravity);
  viewer.ngui->addVariable("Time Step",           params->time_step);

  viewer.ngui->addWidget("Kernel Size", makeSlider(params->kernel_radius, panel));

  wrapper->clock_textbox = new nanogui::TextBox(panel);
  wrapper->clock_textbox->setFontSize(16);
  wrapper->clock_textbox->setValue(to_string(sim->current_time));

  viewer.ngui->addWidget("Time", wrapper->clock_textbox);

  viewer.ngui->addButton("Reset Parameters",
                         [params](){ params->reset(); });

  //////////////////////////////////////////////////////////////////////
  // Rendering.
  //////////////////////////////////////////////////////////////////////
  viewer.ngui->addGroup("Rendering Options");

  viewer.ngui->addVariable("FPS Throttle", params->fps_cap);

  wrapper->fps_textbox = new nanogui::TextBox(panel);
  wrapper->fps_textbox->setFontSize(16);
  wrapper->fps_textbox->setValue(to_string(0.0));

  viewer.ngui->addWidget("FPS", wrapper->fps_textbox);

  viewer.ngui->addVariable("Grid Resolution", params->resolution);
  viewer.ngui->addWidget("Offset", makeSlider(params->surface, panel));
  viewer.ngui->addButton("Show Velocity",
                         [params](){ params->view_mode = ViewMode::VELOCITY; });
  viewer.ngui->addButton("Show Density",
                         [params](){ params->view_mode = ViewMode::DENSITY; });
  viewer.ngui->addButton("Show Surface",
                         [params](){ params->view_mode = ViewMode::SURFACE; });

  //////////////////////////////////////////////////////////////////////
  // Simulation controls.
  //////////////////////////////////////////////////////////////////////
  viewer.ngui->addGroup("Simulation Controls");
  viewer.ngui->addButton("Toggle Simulation",
                         [params](){ params->paused ^= true; });
  viewer.ngui->addButton("Reset Simulation",
                         [sim](){ sim->reset(); });

  // Generate widgets.
  viewer.screen->performLayout();

  // Set to Phong rendering.
  viewer.core.show_lines = false;

  return false;
}

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int mod,
              ViewerWrapper *wrapper) {
  Simulation *sim = wrapper->sim;
  Parameters *params = wrapper->sim->params;

  if (key == ' ')
    params->paused ^= true;
  else if (key == 'S')
    params->view_mode = ViewMode::SURFACE;
  else if (key == 'D')
    params->view_mode = ViewMode::DENSITY;
  else if (key == 'V')
    params->view_mode = ViewMode::VELOCITY;
  else if (key == '1')
    params->scene_mode = SceneMode::DROP;
  else if (key == '2')
    params->scene_mode = SceneMode::DAM;
  else if (key == '3')
    params->scene_mode = SceneMode::SLOSH;
  else if (key == 'R')
    sim->reset();

  return false;
}

bool post_draw(igl::viewer::Viewer& viewer, ViewerWrapper *wrapper) {
  Simulation *sim = wrapper->sim;
  Parameters *params = wrapper->sim->params;

  // Take a step.
  if (!params->paused)
    sim->step();

  // Update clocks.
  wrapper->updateFps();
  wrapper->updateClock();

  // Don't need to waste time changing mesh if a short time has passed.
  if (!wrapper->shouldRender() && sim->current_time != 0.0) {
    glfwPostEmptyEvent();
    return false;
  }

  // Get the current mesh of the simulation. C holds scalar values for colors.
  MatrixX3d V;
  MatrixX3i F;
  VectorXd C;

  // C will have 0 rows if using marching cubes.
  sim->render(V, F, C);

  // Bounding boxes.
  MatrixX3d P;
  MatrixX2i E;
  MatrixX3d EC;
  sim->getBounds(P, E, EC);

  // Update the viewer.
  viewer.data.clear();
  viewer.data.set_mesh(V, F);
  viewer.data.set_edges(P, E, EC);

  // Turn the scalars into colors.
  if (C.rows() > 0) {
    MatrixX3d C_jet;

    // Custom jet from blue to white.
    if (params->view_mode == ViewMode::VELOCITY) {
      C_jet.resize(C.rows(), 3);

      RowVector3d white(1.0, 1.0, 1.0);
      RowVector3d blue(0.0, 0.0, 1.0);

      double min_x = 0.0;
      double max_x = params->fluid_velocity_max;

      for (int i = 0; i < C.rows(); i++) {
        double x = C(i);

        // Linearly interpolate.
        double alpha = min(1.0, (x - min_x) / (max_x - min_x));
        C_jet.row(i) = (1.0 - alpha) * blue + alpha * white;
      }
    }
    else {
      igl::jet(C, params->jet_min, params->jet_max, C_jet);
    }

    viewer.data.set_colors(C_jet);
  }

  if (sim->current_time == 0.0)
    viewer.core.align_camera_center(V, F);

  // Hacky signal to force LibIgl to refresh render loop.
  glfwPostEmptyEvent();

  return false;
}

} // end anonymous namespace for LibIGL helpers and callbacks.

ViewerWrapper::ViewerWrapper(Simulation *sim) : sim(sim) {
  // Zero some stuff.
  fps_ = 0.0;
  last_render_time_ = 0.0;
  fps_textbox = nullptr;
  clock_textbox = nullptr;

  // Attach callbacks to viewer, use bind to pass in the wrapper.
  viewer_.callback_key_down = bind(key_down,
                                   placeholders::_1,
                                   placeholders::_2,
                                   placeholders::_3,
                                   this);
  viewer_.callback_init = bind(init, placeholders::_1, this);
  viewer_.callback_post_draw = bind(post_draw, placeholders::_1, this);
}

void ViewerWrapper::start() {
  viewer_.launch();
}

void ViewerWrapper::updateFps() {
  fps_ = 1000.0 / timer.lap();
  fps_textbox->setValue(to_string(fps_));
}

void ViewerWrapper::updateClock() {
  clock_textbox->setValue(to_string(sim->current_time));
}

bool ViewerWrapper::shouldRender() {
  double time_since_last_render = timer.elapsed() - last_render_time_;
  double fps_rate = 1000.0 / time_since_last_render;

  if (fps_rate > sim->params->fps_cap)
    return false;

  last_render_time_ = timer.elapsed();
  return true;
}
