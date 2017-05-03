#ifndef collision_h
#define collision_h

#include <vector>
#include <map>
#include <utility>

#include <Eigen/Core>

#include "particle.h"

class Collision {

public:
  const Particle *hit;

  Collision(const Particle *particle) {
    hit = particle;
  }

};

class BVHNode {

public:
  BVHNode(std::vector<Particle*> particles);
  ~BVHNode() {
    if (left != nullptr) delete left;
    if (right != nullptr) delete right;
  }

  void getCollisions(const Eigen::Vector3d &query, double radius,
                     std::vector<Collision> &collisions) const;

private:
  Eigen::Vector3d upper;
  Eigen::Vector3d lower;

  BVHNode *left;
  BVHNode *right;

  std::vector<Particle*> items;
};

struct vectorDoubleSort {
  bool operator() (const std::pair<Eigen::Vector3d, double> &lhs,
                   const std::pair<Eigen::Vector3d, double> &rhs) const {
    for (int i = 0; i < 3; i++) {
      if (lhs.first(i) < rhs.first(i))
        return true;
      else if (lhs.first(i) > rhs.first(i))
        return false;
    }

    if (lhs.second < rhs.second)
      return true;

    return false;
  }
};

class BVHTree {

public:
  BVHTree(const std::vector<Particle*> &particles);
  ~BVHTree() { delete root; }

  std::vector<Collision> *getCollisions(const Eigen::Vector3d &query, double r);

private:
  BVHNode *root;

  // This is bad, but kind of good.
  std::map<std::pair<const Eigen::Vector3d, double>,
           std::vector<Collision>,
           vectorDoubleSort> cache_;

};

#endif
