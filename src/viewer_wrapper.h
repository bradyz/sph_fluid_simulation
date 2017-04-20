#ifndef VIEWER_WRAPPER_H
#define VIEWER_WRAPPER_H

class Simulation;

class ViewerWrapper {

public:
  ViewerWrapper(Simulation *sim) : sim_(sim) { }
  void start();

private:
  Simulation *sim_;

};

#endif
