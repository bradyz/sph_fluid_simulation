#ifndef VIEWER_WRAPPER_H
#define VIEWER_WRAPPER_H

#include "simulation.h"

class ViewerWrapper {

public:
  ViewerWrapper(Simulation *sim) : sim_(sim) { }
  void start();

private:
  Simulation *sim_;

};

#endif
