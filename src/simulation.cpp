#include "simulation.h"
#include "parameters.h"
#include "collision.h"
#include "scenes.h"

#include <set>
#include <unordered_map>

#include <Eigen/Core>
#include <igl/copyleft/marching_cubes.h>

using namespace std;
using namespace Eigen;

// anonymous namespace for math.
namespace {

double kernel(const Vector3d &x, const Vector3d &x_i, double s) {
  double r = (x - x_i).norm();
  double c = r / s;

  double w = 1.0 / (M_PI * pow(s, 3));

  if (0.0 <= c && c <= 1.0)
    w *= 1.0 - 3.0 / 2.0 * pow(c, 2) + 3.0 / 4.0 * pow(c, 3);
  else if (1.0 <= c && c <= 2.0)
    w *= 1.0 / 4.0 * pow(2.0 - c, 3);
  else
    w *= 0.0;

  return w;
}

Vector3d kernelGradient(const Vector3d &x, const Vector3d &x_i, double s) {
  double r = (x - x_i).norm();
  double c = r / s;

  double dwdr = 3.0 / (M_PI * pow(s, 4));

  if (0.0 <= c && c <= 1.0)
    dwdr *= c * (-1.0 + 3.0 / 4.0 * c);
  else if (1.0 <= c && c <= 2.0)
    dwdr *= -1.0 / 4.0 * pow(2.0 - c, 2);
  else
    dwdr *= 0.0;

  return dwdr * (x - x_i);
}

double kernelLaplacian(const Vector3d &x, const Vector3d &x_i, double s) {
  double r = (x - x_i).norm();
  double c = r / s;

  double dw2dr = 1.0 / (M_PI * pow(s, 5));

  if (0.0 <= c && c <= 1.0)
    dw2dr *= 3.0 * (-1.0 + 3.0 / 2.0 * c);
  else if (1.0 <= c && c <= 2.0)
    dw2dr *= 3.0 / 2.0 * (2.0 - c);
  else
    dw2dr *= 0.0;

  return dw2dr;
}

} // end anonymous namespace for math.

void Simulation::initialize() {
  cout << "Initializing simulation." << endl;

  Scenes::dropOnPlane(params, particles_, bounds_);
  bvh_ = new BVHTree(particles_);

  // Reverse mapping.
  for (int i = 0; i < particles_.size(); i++)
    particle_to_index_[particles_[i]] = i;

  current_time = 0.0;

  cout << "Total particles: " << particles_.size() << endl;
}

void Simulation::reset() {
  cout << "Resetting simulation." << endl;

  delete bvh_;

  for (Particle *particle : particles_)
    delete particle;

  for (BoundingBox *box : bounds_)
    delete box;

  // Reset the containers.
  particles_.clear();
  bounds_.clear();

  // Reinitialize.
  initialize();
}

void Simulation::step() {
  int n = particles_.size();
  double h = params->time_step;

  // Precondition check.
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 3; j++) {
      if (isnan(particles_[i]->c(j)) || isnan(particles_[i]->v(j))) {
        cerr << "NaNs in particle attributes." << endl;
        cout << "c: " << particles_[i]->c.transpose() << endl;
        cout << "v: " << particles_[i]->v.transpose() << endl;
        exit(1);
      }
    }
  }

  // Velocity Verlet (update position, then velocity).
  for (int i = 0; i < n; i++)
    particles_[i]->c += h * particles_[i]->v;

  // Clean up previous timestep tree.
  if (bvh_ != nullptr)
    delete bvh_;

  bvh_ = new BVHTree(particles_);

  updateDensities();
  applyImpulses();

  // F = -dV'.
  VectorXd forces = getForces();

  // F = ma.
  for (int i = 0; i < n; i++)
    particles_[i]->v += h / particles_[i]->rho * forces.segment<3>(i * 3);

  // Update the clock.
  current_time += h;
}

// Sample the fluid at a single point
double Simulation::getScore(const Vector3d& q) const {
  double score = 0.0;

  double r = params->kernel_radius;
  vector<Collision> *collisions = bvh_->getCollisions(q, r);

  for (Collision &collision : *collisions) {
    int b = particle_to_index_.at(collision.hit);

    double v = particles_[b]->getVolume();
    double w = kernel(q, particles_[b]->c, params->kernel_radius);

    score += v * w;
  }

  return params->surface - score;
}

// Samples the balls at many points
void Simulation::sampleFluid(VectorXd &S, MatrixX3d &P, const int& res) const {
  double b = params->boundary_max;

  S.resize(res * res * res);
  P.resize(res * res * res, 3);

  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      for (int k = 0; k < res; ++k) {
        const double x = b * i / res;
        const double y = b * j / res;
        const double z = b * k / res;

        P.row(i + res * (j + res * k)) = RowVector3d(x, y, z);
        S(i + res * (j + res * k)) = getScore(RowVector3d(x, y, z));
      }
    }
  }
}

void Simulation::render(MatrixX3d &V, MatrixX3i &F, VectorXd &C) const {
  if (params->view_mode == ViewMode::SURFACE) {
    C.resize(0);

    VectorXd samples;
    MatrixX3d positions;
    int res = params->resolution;

    sampleFluid(samples, positions, res);

    igl::copyleft::marching_cubes(samples, positions, res, res, res, V, F);

    return;
  }

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

    for (int i = total_v; i < total_v + nb_v; i++) {
      if (params->view_mode == ViewMode::DENSITY)
        C(i) = log(particle->rho);
      else if (params->view_mode == ViewMode::VELOCITY)
        C(i) = particle->v.squaredNorm();
    }

    V.block(total_v, 0, nb_v, 3) = particle->getV();
    F.block(total_f, 0, nb_f, 3) = particle->getF().array() + total_v;

    // Get the next offset.
    total_v += nb_v;
    total_f += nb_f;
  }
}

void Simulation::getBounds(MatrixX3d &V, MatrixX2i &E, MatrixX3d &C) const {
  // Find out the total number of vertex and edge rows needed.
  int total_v_rows = 0;
  int total_e_rows = 0;

  for (const BoundingBox *box : bounds_) {
    total_v_rows += box->getV().rows();
    total_e_rows += box->getE().rows();
  }

  // Reallocate to the proper amount of space.
  V.resize(total_v_rows, 3);
  V.setZero();
  E.resize(total_e_rows, 3);
  E.setZero();
  C.resize(total_v_rows, 3);
  C.rowwise() = RowVector3d(1.0, 0.5, 0.5);

  // The current block of vertices and faces.
  int total_v = 0;
  int total_e = 0;

  // Add each mesh to be rendered.
  for (const BoundingBox *box : bounds_) {
    int nb_v = box->getV().rows();
    int nb_e = box->getE().rows();

    V.block(total_v, 0, nb_v, 3) = box->getV();
    E.block(total_e, 0, nb_e, 3) = box->getE().array() + total_e;

    // Get the next offset.
    total_v += nb_v;
    total_e += nb_e;
  }
}

void Simulation::applyImpulses() {
  int n = particles_.size();

  // Collisions resolved.
  unordered_map<int, unordered_map<int, bool> > visited;

  for (int a = 0; a < n; a++) {
    double r = particles_[a]->r;
    vector<Collision> *collisions = bvh_->getCollisions(particles_[a]->c, r);

    for (Collision &collision : *collisions) {
      int b = particle_to_index_.at(collision.hit);

      // Ignore self collision and previously resolved collisions.
      if (a == b)
        continue;
      else if (visited[a][b])
        continue;

      // Mark collision as resolved.
      visited[a][b] = true;
      visited[b][a] = true;

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

void Simulation::updateDensities() {
  for (int a = 0; a < particles_.size(); a++) {
    double rho_i = 0.0;

    double r = params->kernel_radius;
    vector<Collision> *collisions  = bvh_->getCollisions(particles_[a]->c, r);

    for (Collision &collision : *collisions) {
      int b = particle_to_index_.at(collision.hit);

      double m = particles_[b]->m;
      double w = kernel(particles_[a]->c, particles_[b]->c, params->kernel_radius);

      rho_i += m * w;
    }

    particles_[a]->rho = rho_i;
  }
}

VectorXd Simulation::getForces() const {
  // Accumulation of different forces.
  VectorXd force(particles_.size() * 3);
  force.setZero();

  getGravityForce(force);
  getBoundaryForce(force);
  getPressureForce(force);
  getViscosityForce(force);

  return force;
}

void Simulation::getGravityForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++)
    force(i * 3 + 1) += particles_[i]->m * params->gravity;
}

void Simulation::getBoundaryForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    for (const BoundingBox *box : bounds_) {
      Vector3d u = box->intersects(particles_[i]->c);
      double c = u.squaredNorm();

      // Not in the box.
      if (c <= 1e-8)
        continue;

      Vector3d u_hat = u.normalized();
      double m = particles_[i]->m;
      double k = params->penalty_coefficient;

      force.segment<3>(i * 3) += k * m * c * u_hat;
    }
  }
}

void Simulation::getPressureForce(VectorXd &force) const {
  for (int a = 0 ; a < particles_.size(); a++) {
    Vector3d f_i(0.0, 0.0, 0.0);

    double r = params->kernel_radius;
    vector<Collision> *collisions = bvh_->getCollisions(particles_[a]->c, r);

    for (Collision &collision : *collisions) {
      int b = particle_to_index_.at(collision.hit);

      double m = particles_[b]->m;

      double p_i = particles_[a]->getPressure();
      double p_j = particles_[b]->getPressure();
      double rho_i = particles_[a]->rho;
      double rho_j = particles_[b]->rho;

      Vector3d dw = kernelGradient(particles_[a]->c, particles_[b]->c,
                                   params->kernel_radius);

      f_i += -m * (p_i + p_j) / (2.0 * rho_i * rho_j) * dw;
    }

    force.segment<3>(a * 3) += f_i;
  }
}

void Simulation::getViscosityForce(VectorXd &force) const {
  for (int a = 0; a < particles_.size(); a++) {
    Vector3d f_i(0.0, 0.0, 0.0);

    double r = params->kernel_radius;
    vector<Collision> *collisions = bvh_->getCollisions(particles_[a]->c, r);

    for (Collision &collision : *collisions) {
      int b = particle_to_index_.at(collision.hit);

      double m = particles_[b]->m;

      double mu_i = particles_[a]->mu;
      double mu_j = particles_[b]->mu;

      Vector3d v_i = particles_[a]->v;
      Vector3d v_j = particles_[b]->v;

      double rho_j = particles_[b]->rho;
      double dw2dr = kernelLaplacian(particles_[a]->c, particles_[b]->c,
                                     params->kernel_radius);

      f_i += (mu_i + mu_j) / 2.0 * m * (v_j - v_i) / rho_j * dw2dr;
    }

    force.segment<3>(a * 3) += f_i;
  }
}

Simulation::~Simulation() {
  delete params;
  delete bvh_;

  for (Particle *particle : particles_)
    delete particle;

  for (BoundingBox *box : bounds_)
    delete box;
}
