#include "collision.h"
#include "particle.h"

#include <set>
#include <vector>
#include <unordered_map>

using namespace std;
using namespace Eigen;

// anonymous for intersections.
namespace {

bool sphereSphereIntersect(const Vector3d &c1, double r1,
                           const Vector3d &c2, double r2) {
  return ((c1 - c2).squaredNorm() < (r1 + r2) * (r1 + r2));
}

} // end anonymous for intersections.

// TODO: construct tree.
BVHNode::BVHNode(const vector<Particle*> &particles) {
  upper.setZero();
  lower.setZero();

  left = NULL;
  right = NULL;

  items = particles;
}

// TODO: fast version.
void BVHNode::getCollisions(const Particle *lhs, double radius,
                            vector<Collision> &collisions) const {
  for (const Particle *rhs : items) {
    if (lhs == rhs)
      continue;

    if (sphereSphereIntersect(lhs->c, radius, rhs->c, rhs->r))
      collisions.push_back(Collision(lhs, rhs));
  }
}

BVHTree::BVHTree(const vector<Particle*> &particles) {
  root = new BVHNode(particles);
}

void BVHTree::getCollisions(const Particle *particle, double radius,
                            vector<Collision> &collisions) const {
  root->getCollisions(particle, radius, collisions);
}
