#ifndef VIEWER_WRAPPER_H
#define VIEWER_WRAPPER_H

#include "simulation.h"
#include "timer.h"

#include <igl/viewer/Viewer.h>

#include <nanogui/textbox.h>

// This class is ridiculously filled with hacks.
// TODO: refactor to extend igl::Viewer, not gonna happen though.
class ViewerWrapper {

public:
  ViewerWrapper(Simulation *sim);

  void start();

  // Sets the textboxes in the gui.
  void updateFps();
  void updateClock();

  // Cap framerate to save some performance.
  bool shouldRender();

  // Has to be public so the LibIGL render loop can access these.
  Timer timer;
  Simulation *sim;

  nanogui::TextBox *fps_textbox;
  nanogui::TextBox *clock_textbox;

private:
  igl::viewer::Viewer viewer_;

  double fps_;
  double last_render_time_;

};

#endif
