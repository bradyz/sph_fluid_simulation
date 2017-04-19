#ifndef collision_h
#define collision_h

#include <vector>

#include <Eigen/Core>

#include "particle.h"

class Collision {

public:
  const Particle *a;
  const Particle *b;

  Collision(const Particle *x, const Particle *y) {
    a = (x > y) ? x : y;
    b = (x > y) ? y : x;
  }

  bool operator<(const Collision &rhs) const {
    if (a < rhs.a)
      return true;
    else if (a > rhs.a)
      return false;
    return (b < rhs.b);
  }

};

class BVHNode {

public:
  BVHNode(const std::vector<Particle*> &particles);
  ~BVHNode() {
    if (left != NULL) delete left;
    if (right != NULL) delete right;
  }

  void getCollisions(const Particle *particle, double radius,
                     std::vector<Collision> &collisions) const;

private:
  Eigen::Vector3d upper;
  Eigen::Vector3d lower;

  BVHNode *left;
  BVHNode *right;

  std::vector<Particle*> items;

};

class BVHTree {

public:
  BVHTree(const std::vector<Particle*> &particles);
  ~BVHTree() { delete root; }

  void getCollisions(const Particle *particle, double radius,
                     std::vector<Collision> &collisions) const;

private:
  BVHNode *root;

};


#endif
