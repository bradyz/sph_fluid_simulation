#ifndef SIMULATION_H
#define SIMULATION_H

#include "mesh.h"
#include "parameters.h"
#include "particle.h"

#include <vector>

class Simulation {

public:
  Parameters *params;

  Simulation() : params(new Parameters()) { }

  void initialize();
  void start();
  void step();
  void reset();

  // Puts the entire simulation into the two matrices (world coordinates).
  void render(Eigen::MatrixX3d &V, Eigen::MatrixX3i &F) const;

  ~Simulation();

private:
  std::vector<Mesh*> meshes_;
  std::vector<Particle*> particles_;

};

#endif
