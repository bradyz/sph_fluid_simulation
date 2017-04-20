#ifndef PARTICLE_H
#define PARTICLE_H

#include "mesh.h"

#include <Eigen/Core>

class Particle {

public:
  double r;                                         // scale.
  double m;                                         // mass, kg.
  Eigen::Vector3d c;                                // position, m.
  Eigen::Vector3d v;                                // velocity, m / s.
  Eigen::Vector3d f;                                // body forces, N / m^3.

  double k;                                         // gas constant, N m / kg.

  double rho_0;                                     // rest density, kg / m^3.
  double rho;                                       // density, kg / m^3.

  double mu;                                        // viscocity, N s / m^3.

  Particle(const Mesh *mesh) : mesh_(mesh) { }

  double getPressure() const { return k * (rho - rho_0); }

  const Eigen::MatrixX3d getV() const {
    return (mesh_->getV() * r).rowwise() + c.transpose();
  }

  const Eigen::MatrixX3i& getF() const { return mesh_->getF(); }

private:
  const Mesh *mesh_;

};

#endif
