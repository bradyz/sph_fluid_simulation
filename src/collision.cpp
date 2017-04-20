#include "collision.h"
#include "particle.h"

#include <algorithm>
#include <set>
#include <vector>
#include <unordered_map>

using namespace std;
using namespace Eigen;

// anonymous for intersections.
namespace {

inline double squared(double v) { return v * v; }

bool sphereSphereIntersect(const Vector3d &c1, double r1,
                           const Vector3d &c2, double r2) {
  return ((c1 - c2).squaredNorm() < (r1 + r2) * (r1 + r2));
}

bool sphereCubeIntersect(const Vector3d &lower, const Vector3d &upper,
                         const Vector3d &c, double radius) {
    double dist_squared = radius * radius;
    double cube_sphere_dist = 0.0;

    if (c(0) < lower(0)) cube_sphere_dist += squared(c(0) - lower(0));
    else if (c(0) > upper(0)) cube_sphere_dist += squared(c(0) - upper(0));
    if (c(1) < lower(1)) cube_sphere_dist += squared(c(1) - lower(1));
    else if (c(1) > upper(1)) cube_sphere_dist += squared(c(1) - upper(1));
    if (c(2) < lower(2)) cube_sphere_dist += squared(c(2) - lower(2));
    else if (c(2) > upper(2)) cube_sphere_dist += squared(c(2) - upper(2));

    return dist_squared >= cube_sphere_dist;
}

} // end anonymous for intersections.

BVHNode::BVHNode(vector<Particle*> particles) {
  items = particles;

  left = nullptr;
  right = nullptr;

  upper.setZero();
  lower.setZero();

  for (Particle *particle : particles) {
    for (int j = 0; j < 3; j++) {
      upper(j) = max(upper(j), particle->c(j));
      lower(j) = min(lower(j), particle->c(j));
    }
  }

  // Leaf.
  if (particles.size() <= 1)
    return;

  int axis = 0;
  for (int i = 0; i < 3; i++) {
    if (upper(i) - lower(i) > upper(axis) - lower(axis))
      axis = i;
  }

  sort(particles.begin(), particles.end(),
       [&](const Particle *lhs, const Particle *rhs) {
         return lhs->c(axis) - rhs->c(axis);
  });

  int split = particles.size() / 2;

  vector<Particle*> left_particles;
  vector<Particle*> right_particles;

  for (int i = 0; i < split; i++)
    left_particles.push_back(particles[i]);

  for (int i = split; i < particles.size(); i++)
    right_particles.push_back(particles[i]);

  left = new BVHNode(left_particles);
  right = new BVHNode(right_particles);
}

void BVHNode::getCollisions(const Particle *lhs, double radius,
                            vector<Collision> &collisions) const {
  if (!sphereCubeIntersect(lower, upper, lhs->c, radius))
    return;

  if (left == nullptr && right == nullptr) {
    for (const Particle *rhs : items) {
      if (sphereSphereIntersect(lhs->c, radius, rhs->c, rhs->r))
        collisions.push_back(Collision(lhs, rhs));
    }
  }
  else {
    left->getCollisions(lhs, radius, collisions);
    right->getCollisions(lhs, radius, collisions);
  }
}

BVHTree::BVHTree(const vector<Particle*> &particles) {
  root = new BVHNode(particles);
}

void BVHTree::getCollisions(const Particle *particle, double radius,
                            vector<Collision> &collisions) const {
  root->getCollisions(particle, radius, collisions);
}
