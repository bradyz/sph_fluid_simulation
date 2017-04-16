#include "simulation.h"

int main (int argc, char* argv[]) {
  Simulation *sim = new Simulation();

  sim->initialize();
  sim->start();

  delete sim;
}
