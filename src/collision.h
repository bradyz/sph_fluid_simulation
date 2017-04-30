#ifndef collision_h
#define collision_h

#include <vector>

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

class BVHTree {

public:
  BVHTree(const std::vector<Particle*> &particles);
  ~BVHTree() { delete root; }

  void getCollisions(const Eigen::Vector3d &query, double radius,
                     std::vector<Collision> &collisions) const;

private:
  BVHNode *root;
};


#endif
