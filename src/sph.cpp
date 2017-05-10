#include "simulation.h"
#include "viewer_wrapper.h"

#include <thread>

void simulationLoop(Simulation *sim) {
  while (true) {
    if (!sim->params->paused)
      sim->step();
  }
}

int main (int argc, char* argv[]) {
  Simulation *sim = new Simulation();
  sim->initialize();

  std::thread sim_thread(simulationLoop, sim);

  ViewerWrapper viewer(sim);
  viewer.start();

  delete sim;
}
