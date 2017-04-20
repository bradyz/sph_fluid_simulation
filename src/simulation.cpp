#include "simulation.h"
#include "parameters.h"
#include "collision.h"

#include <set>
#include <unordered_map>

using namespace std;
using namespace Eigen;

void Simulation::initialize() {
  cout << "Initializing simulation." << endl;

  // The sphere mesh has a radius of 0.1.
  meshes_.push_back(new Mesh("../obj/sphere.obj", 10.0));

  for (int i = 0; i < params->nb_particles; i++) {
    for (int j = 0; j < params->nb_particles; j++) {
      for (int k = 0; k < params->nb_particles; k++) {
        Particle *particle = new Particle(meshes_.front());

        particle->m = params->mass;
        particle->r = params->radius;
        particle->c = Vector3d(3.0 * i * particle->r,
                               3.0 * j * particle->r,
                               3.0 * k * particle->r);
        particle->c += Vector3d(2.5, 0.5, 2.5);
        if (j % 2 == 0)
          particle->c += Vector3d(0.5, 0.0, 0.0);

        particle->v = Vector3d(0.0, 0.0, 0.0);
        particle->k = params->gas_constant;
        particle->rho_0 = params->density;
        particle->rho = params->density;

        particles_.push_back(particle);
      }
    }
  }

  BVHTree tree(particles_);
  updateDensities(tree);

  for (Particle *particle : particles_)
    particle->rho_0 = particle->rho;

  cout << "Total particles: " << particles_.size() << endl;
}

void Simulation::reset() {
  cout << "Resetting simulation." << endl;

  // Clean up memory.
  for (Particle *particle : particles_)
    delete particle;
  for (Mesh *mesh : meshes_)
    delete mesh;

  // Reset the containers.
  particles_.clear();
  meshes_.clear();

  // Reinitialize.
  initialize();
}

void Simulation::step() {
  int n = particles_.size();
  double h = params->time_step;

  // Velocity Verlet (update position, then velocity).
  for (int i = 0; i < n; i++) {
    particles_[i]->c += h * particles_[i]->v;

    if (isnan(particles_[i]->c(0))) {
      cerr << "NaNs in particle positions." << endl;
      cout << particles_[i]->c.transpose() << endl;
      cout << particles_[i]->v.transpose() << endl;
      exit(1);
    }
  }

  // Build BVH Tree.
  BVHTree tree(particles_);

  updateDensities(tree);
  applyImpulses(tree);

  // F = -dV'.
  VectorXd forces = getForces(tree);

  // F = ma.
  for (int i = 0; i < n; i++)
    particles_[i]->v += h / particles_[i]->rho * forces.segment<3>(i * 3);

  // Update the clock.
  current_time += h;
}

void Simulation::render(MatrixX3d &V, MatrixX3i &F, VectorXd &C) const {
  // Find out the total number of vertex and face rows needed.
  int total_v_rows = 0;
  int total_f_rows = 0;

  for (const Particle *particle : particles_) {
    total_v_rows += particle->getV().rows();
    total_f_rows += particle->getF().rows();
  }

  // Reallocate to the proper amount of space.
  V.resize(total_v_rows, 3);
  V.setZero();
  F.resize(total_f_rows, 3);
  F.setZero();
  C.resize(total_v_rows);
  C.setZero();

  // The current block of vertices and faces.
  int total_v = 0;
  int total_f = 0;

  // Add each mesh to be rendered.
  for (const Particle *particle : particles_) {
    int nb_v = particle->getV().rows();
    int nb_f = particle->getF().rows();

    for (int i = total_v; i < total_v + nb_v; i++)
      C(i) = log(particle->rho);

    V.block(total_v, 0, nb_v, 3) = particle->getV();
    F.block(total_f, 0, nb_f, 3) = particle->getF().array() + total_v;

    // Get the next offset.
    total_v += nb_v;
    total_f += nb_f;
  }
}

void Simulation::getBounds(MatrixX3d &V, MatrixX2i &E, MatrixX3d &C) const {
  double b = params->boundary_max;

  V.resize(8, 3);
  E.resize(12, 2);
  C.resize(12, 3);

  V.row(0) = RowVector3d(0.0, 0.0, 0.0);
  V.row(1) = RowVector3d(  b, 0.0, 0.0);
  V.row(2) = RowVector3d(0.0, 0.0,   b);
  V.row(3) = RowVector3d(  b, 0.0,   b);
  V.row(4) = RowVector3d(0.0,   b, 0.0);
  V.row(5) = RowVector3d(  b,   b, 0.0);
  V.row(6) = RowVector3d(0.0,   b,   b);
  V.row(7) = RowVector3d(  b,   b,   b);

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

  C.rowwise() = RowVector3d(1.0, 0.5, 0.5);
}

void Simulation::applyImpulses(BVHTree &tree) {
  // Reverse mapping.
  unordered_map<const Particle*, int> particle_to_index;
  for (int i = 0; i < particles_.size(); i++)
    particle_to_index[particles_[i]] = i;

  // Collisions resolved.
  set<Collision> visited;

  for (const Particle *particle : particles_) {
    vector<Collision> collisions;
    tree.getCollisions(particle, particle->r, collisions);

    for (Collision &collision : collisions) {
      // Have resolved collision before.
      if (visited.find(collision) != visited.end())
        continue;

      // Mark collision as resolved.
      visited.insert(collision);

      int a = particle_to_index[collision.a];
      int b = particle_to_index[collision.b];

      // Ignore self collision.
      if (a == b)
        continue;

      // Vector from b to a.
      Vector3d n_hat = (particles_[a]->c - particles_[b]->c).normalized();
      double v_minus = (particles_[a]->v - particles_[b]->v).dot(n_hat);

      // Collision has already been resolved in previous timestep.
      if (v_minus > 0.0)
        continue;

      // Want v_new = -c v_old.
      double m1 = particles_[a]->m;
      double m2 = particles_[b]->m;
      double c = params->coefficient_of_restitution;
      double alpha = -(1.0 + c) * v_minus / (1.0 / m1 + 1.0 / m2);
      Vector3d J = alpha * n_hat;

      // Apply impulse.
      particles_[a]->v += 1.0 / m1 * J;
      particles_[b]->v -= 1.0 / m2 * J;
    }
  }
}

void Simulation::updateDensities(BVHTree &tree) {
  // Reverse mapping.
  unordered_map<const Particle*, int> particle_to_index;
  for (int i = 0; i < particles_.size(); i++)
    particle_to_index[particles_[i]] = i;

  for (int i = 0 ; i < particles_.size(); i++) {
    double rho_i = 0.0;

    vector<Collision> collisions;
    tree.getCollisions(particles_[i], params->kernel_radius, collisions);

    for (Collision &collision : collisions) {
      int a = particle_to_index[collision.a];
      int b = particle_to_index[collision.b];

      double m = particles_[b]->m;
      double r = (particles_[a]->c - particles_[b]->c).norm();
      double s = params->kernel_radius;
      double c = r / s;

      double w = 1.0 / (M_PI * pow(s, 3));

      if (0.0 <= c && c <= 1.0)
        w *= 1.0 - 3.0 / 2.0 * pow(c, 2) + 3.0 / 4.0 * pow(c, 3);
      else if (1.0 <= c && c <= 2.0)
        w *= 1.0 / 4.0 * pow(2.0 - c, 3);
      else
        w *= 0.0;

      rho_i += m * w;
    }

    particles_[i]->rho = rho_i;
  }
}

VectorXd Simulation::getForces(BVHTree &tree) const {
  // Accumulation of different forces.
  VectorXd force(particles_.size() * 3);
  force.setZero();

  getGravityForce(force);
  getBoundaryForce(force);
  getPressureForce(force, tree);
  // getViscosityForce(force);

  return force;
}

void Simulation::getGravityForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++)
    force(i * 3 + 1) += particles_[i]->m * params->gravity;
}

void Simulation::getBoundaryForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    for (int j = 0; j < 3; j++) {
      double m = particles_[i]->m;
      double p = particles_[i]->c(j);
      double k = params->penalty_coefficient;
      double b_min = params->boundary_min;
      double b_max = params->boundary_max;

      if (p < b_min)
        force(i * 3 + j) += (b_min - p) * k * m;
      if (p > b_max)
        force(i * 3 + j) += (b_max - p) * k * m;
    }
  }
}

void Simulation::getPressureForce(VectorXd &force, BVHTree &tree) const {
  // Reverse mapping.
  unordered_map<const Particle*, int> particle_to_index;
  for (int i = 0; i < particles_.size(); i++)
    particle_to_index[particles_[i]] = i;

  for (int i = 0 ; i < particles_.size(); i++) {
    Vector3d f_i(0.0, 0.0, 0.0);

    vector<Collision> collisions;
    tree.getCollisions(particles_[i], params->kernel_radius, collisions);

    for (Collision &collision : collisions) {
      int a = particle_to_index[collision.a];
      int b = particle_to_index[collision.b];

      if (particles_[a] != particles_[i])
        swap(a, b);

      double m = particles_[b]->m;

      double p_i = particles_[a]->getPressure();
      double p_j = particles_[b]->getPressure();
      double rho_i = particles_[a]->rho;
      double rho_j = particles_[b]->rho;

      Vector3d u = particles_[a]->c - particles_[b]->c;
      double r = u.norm();
      double s = params->kernel_radius;
      double c = r / s;

      // Gradient of kernel.
      double dwdr = 3.0 / (M_PI * pow(s, 4));

      if (0.0 <= c && c <= 1.0)
        dwdr *= c * (-1.0 + 3.0 / 4.0 * c);
      else if (1.0 <= c && c <= 2.0)
        dwdr *= -1.0 / 4.0 * pow(2.0 - c, 2);
      else
        dwdr *= 0.0;

      f_i += -m * (p_i + p_j) / (2.0 * rho_i * rho_j) * dwdr * u;
    }

    force.segment<3>(i * 3) += f_i;
  }
}

void Simulation::getViscosityForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    Vector3d f_i(0.0, 0.0, 0.0);

    for (int j = 0; j < particles_.size(); j++) {
      double m = particles_[j]->m;

      double mu_i = particles_[i]->mu;
      double mu_j = particles_[j]->mu;

      Vector3d v_i = particles_[i]->v;
      Vector3d v_j = particles_[j]->v;

      double rho_j = particles_[j]->rho;

      Vector3d u = particles_[i]->c - particles_[j]->c;
      double r = u.norm();
      double s = params->kernel_radius;
      double c = r / s;

      double dw2dr = 1.0 / (M_PI * pow(s, 5));

      if (0.0 <= c && c <= 1.0)
        dw2dr *= 3.0 * (-1.0 + 3.0 / 2.0 * c);
      else if (1.0 <= c && c <= 2.0)
        dw2dr *= 3.0 / 2.0 * (2.0 - c);
      else
        dw2dr *= 0.0;

      // THIS LINE IS FUCKED
      f_i += (mu_i + mu_j) / 2.0 * m * (v_j - v_i) / rho_j * dw2dr;
    }

    force.segment<3>(i * 3) += f_i;
  }
}

Simulation::~Simulation() {
  delete params;

  for (Particle *particle : particles_)
    delete particle;
  for (Mesh *mesh : meshes_)
    delete mesh;
}
