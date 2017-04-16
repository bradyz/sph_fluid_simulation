#ifndef MESH_H
#define MESH_H

#include <Eigen/Core>

#include <igl/readOBJ.h>

class Mesh {

public:
  Mesh(const char *filename) {
    igl::readOBJ(filename, V_, F_);
  }

  const Eigen::MatrixX3d &getV() const { return V_; }
  const Eigen::MatrixX3i &getF() const { return F_; }

private:
  Eigen::MatrixX3d V_;
  Eigen::MatrixX3i F_;

};

#endif
