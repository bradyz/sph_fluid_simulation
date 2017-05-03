#include "bounding_box.h"

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

Vector3d BoundingBox::intersects(const Vector3d &p) const {
  // Ignore if not in box.
  for (int i = 0; i < 3; i++) {
    if (p(i) < lower_(i) || p(i) > upper_(i))
      return Vector3d(0.0, 0.0, 0.0);
  }

  // Best vector to follow to get out of the box.
  Vector3d result(1.0, 1.0, 1.0);

  // Look for minimum distance to each side of the box.
  for (int i = 0; i < 3; i++) {
    if (p(i) < lower_(i) || p(i) > upper_(i))
      continue;

    Vector3d tmp(0.0, 0.0, 0.0);
    tmp(i) = 1.0;

    double dist_from_lower = p(i) - lower_(i);
    double dist_from_upper = upper_(i) - p(i);

    if (dist_from_lower < dist_from_upper)
      tmp *= -dist_from_lower;
    else
      tmp *= dist_from_upper;

    // Shorter path out.
    if (tmp.squaredNorm() < result.squaredNorm())
      result = tmp;
  }

  return result;
}

MatrixX3d BoundingBox::getV() const {
  MatrixX3d V(8, 3);

  V.row(0) = RowVector3d(lower_(0), lower_(1), lower_(2));
  V.row(1) = RowVector3d(upper_(0), lower_(1), lower_(2));
  V.row(2) = RowVector3d(lower_(0), upper_(1), lower_(2));
  V.row(3) = RowVector3d(upper_(0), upper_(1), lower_(2));
  V.row(4) = RowVector3d(lower_(0), lower_(1), upper_(2));
  V.row(5) = RowVector3d(upper_(0), lower_(1), upper_(2));
  V.row(6) = RowVector3d(lower_(0), upper_(1), upper_(2));
  V.row(7) = RowVector3d(upper_(0), upper_(1), upper_(2));

  return V;
}

MatrixX2i BoundingBox::getE() const {
  MatrixX2i E(12, 2);

  E.row(0)  = RowVector2i(0, 1);
  E.row(1)  = RowVector2i(0, 2);
  E.row(2)  = RowVector2i(1, 3);
  E.row(3)  = RowVector2i(2, 3);
  E.row(4)  = RowVector2i(0, 4);
  E.row(5)  = RowVector2i(1, 5);
  E.row(6)  = RowVector2i(2, 6);
  E.row(7)  = RowVector2i(3, 7);
  E.row(8)  = RowVector2i(4, 5);
  E.row(9)  = RowVector2i(4, 6);
  E.row(10) = RowVector2i(5, 7);
  E.row(11) = RowVector2i(6, 7);

  return E;
}
