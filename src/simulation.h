#ifndef SIMULATION_H
#define SIMULATION_H

#include "mesh.h"
#include "parameters.h"
#include "particle.h"
#include "collision.h"

#include <vector>
#include <unordered_map>

#include <Eigen/Core>

class Simulation {

public:
  Parameters *params;
  double current_time;

  Simulation() : params(new Parameters()), current_time(0.0) { }

  void initialize();
  void step();
  void reset();

  // Samples the balls at many points
  double getScore(const Eigen::Vector3d &Q) const;
  void sampleFluid(Eigen::VectorXd &S, Eigen::MatrixX3d &P, const int& res) const;

  // Puts the entire simulation into the two matrices (world coordinates).
  void render(Eigen::MatrixX3d &V, Eigen::MatrixX3i &F, Eigen::VectorXd &C) const;
  void getBounds(Eigen::MatrixX3d &V, Eigen::MatrixX2i &E,
                 Eigen::MatrixX3d &C) const;

  void applyImpulses();
  void updateDensities();

  Eigen::VectorXd getForces() const;
  void getGravityForce(Eigen::VectorXd &force) const;
  void getBoundaryForce(Eigen::VectorXd &force) const;
  void getPressureForce(Eigen::VectorXd &force) const;
  void getViscosityForce(Eigen::VectorXd &force) const;

  ~Simulation();

private:
  std::vector<Mesh*> meshes_;
  std::vector<Particle*> particles_;

  BVHTree *bvh_tree_;
  std::unordered_map<const Particle*, int> particle_to_index_;

};

#endif
