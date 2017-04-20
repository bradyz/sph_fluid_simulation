#include "simulation.h"
#include "viewer_wrapper.h"

int main (int argc, char* argv[]) {
  Simulation *sim = new Simulation();
  sim->initialize();

  ViewerWrapper viewer(sim);
  viewer.start();

  delete sim;
}
