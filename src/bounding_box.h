#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <Eigen/Core>

class BoundingBox {

public:
  BoundingBox(const Eigen::Vector3d &lower, const Eigen::Vector3d &upper) :
    lower_(lower), upper_(upper) { }

  // Vector to get out, norm of vector is amount of penetration.
  Eigen::Vector3d intersects(const Eigen::Vector3d &point) const;

  // Used for rendering.
  Eigen::MatrixX3d getV() const;
  Eigen::MatrixX2i getE() const;

private:
  Eigen::Vector3d lower_;
  Eigen::Vector3d upper_;

};

#endif
