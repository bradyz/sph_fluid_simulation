#ifndef SIMULATION_H
#define SIMULATION_H

#include "mesh.h"
#include "parameters.h"
#include "particle.h"

#include <vector>

#include <Eigen/Core>

class Simulation {

public:
  Parameters *params;
  double current_time;

  Simulation() : params(new Parameters()), current_time(0.0) { }

  void initialize();
  void start();
  void step();
  void reset();

  // Puts the entire simulation into the two matrices (world coordinates).
  void render(Eigen::MatrixX3d &V, Eigen::MatrixX3i &F) const;

  Eigen::VectorXd getForces() const;

  void getGravityForce(Eigen::VectorXd &force) const;
  void getBoundaryForce(Eigen::VectorXd &force) const;
  void getCollisionForce(Eigen::VectorXd &force) const;
  void getPressureForce(Eigen::VectorXd &force) const;
  void getViscosityForce(Eigen::VectorXd &force) const;

  ~Simulation();

private:
  std::vector<Mesh*> meshes_;
  std::vector<Particle*> particles_;

};

#endif
