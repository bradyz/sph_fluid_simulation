#include "simulation.h"
#include "parameters.h"
#include "collision.h"

#include <functional>
#include <set>
#include <unordered_map>

#include <igl/viewer/Viewer.h>

using namespace std;
using namespace Eigen;

// anonymous namespace to handle LibIGL viewer.
namespace {

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int mod,
              Simulation *sim) {
  if (key == ' ')
    sim->params->paused = !sim->params->paused;
  else if (key == 'R')
    sim->reset();

  return false;
}

bool init(igl::viewer::Viewer& viewer, Simulation *sim) {
  viewer.ngui->addWindow(Vector2i(220, 10), "SPH Fluid Simulation");

  // Simulation environment variables.
  viewer.ngui->addGroup("Simulation Parameters");
  viewer.ngui->addVariable("Number of Particles", sim->params->nb_particles);
  viewer.ngui->addVariable("Radius",              sim->params->radius);
  viewer.ngui->addVariable("Mass",                sim->params->mass);
  viewer.ngui->addVariable("Density",             sim->params->density);
  viewer.ngui->addVariable("Viscocity",           sim->params->viscocity);
  viewer.ngui->addVariable("Gravity",             sim->params->gravity);
  viewer.ngui->addVariable("Time Step",           sim->params->time_step);
  viewer.ngui->addVariable("Current Time",        sim->current_time, false);
  viewer.ngui->addButton("Reset Parameters",
                         [sim](){ sim->params->reset(); });

  // Simulation controls..
  viewer.ngui->addGroup("Simulation Controls");
  viewer.ngui->addButton("Toggle Simulation",
                         [sim](){ sim->params->paused = !sim->params->paused; });
  viewer.ngui->addButton("Reset Simulation",
                         [sim](){ sim->reset(); });

  // Generate widgets.
  viewer.screen->performLayout();

  return false;
}

bool post_draw(igl::viewer::Viewer& viewer, Simulation *sim) {
  // Take a step.
  if (!sim->params->paused)
    sim->step();

  // Get the current mesh of the simulation.
  MatrixX3d V;
  MatrixX3i F;
  sim->render(V, F);

  // Update the viewer.
  viewer.data.clear();
  viewer.data.set_mesh(V, F);

  // Signal to render.
  glfwPostEmptyEvent();

  return false;
}

} // end anonymous namespace to handle LibIGL viewer.

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
        particle->c = Vector3d(2.0 * i * particle->r,
                               2.0 * j * particle->r,
                               2.0 * k * particle->r);
        particle->c += Vector3d(0.2, 0.2, 0.2);
        particle->v = Vector3d(0.0, 0.0, 0.0);
        particle->k = params->gas_constant;
        particle->rho_0 = 1.0;
        particle->rho = 1.0;

        particles_.push_back(particle);
      }
    }
  }

  cout << "Total particles: " << particles_.size() << endl;
}

void Simulation::start() {
  igl::viewer::Viewer viewer;
  viewer.callback_key_down = bind(key_down,
                                  placeholders::_1,
                                  placeholders::_2,
                                  placeholders::_3,
                                  this);
  viewer.callback_init = bind(init, placeholders::_1, this);
  viewer.callback_post_draw = bind(post_draw, placeholders::_1, this);

  // Get all the meshes in the simulation.
  MatrixX3d V;
  MatrixX3i F;
  render(V, F);

  // Update the viewer.
  viewer.data.clear();
  viewer.data.set_mesh(V, F);

  viewer.launch();
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
  for (int i = 0; i < n; i++)
    particles_[i]->c = particles_[i]->c + h * particles_[i]->v;

  // Build BVH Tree.
  BVHTree tree(particles_);
  
  applyImpulses(tree);

  // Update densities.
  for (int i = 0; i < n; i++) {
    break;

    double rho_i = 0.0;

    for (int j = 0; j < n; j++) {
      double m = particles_[j]->m;
      double r = (particles_[i]->c - particles_[j]->c).norm();
      double s = params->kernel_support;
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

  // F = -dV'.
  VectorXd forces = getForces();

  // F = ma.
  for (int i = 0; i < n; i++)
    particles_[i]->v += h / particles_[i]->m * forces.segment<3>(i * 3);

  // Update the clock.
  current_time += h;
}

void Simulation::render(MatrixX3d &V, MatrixX3i &F) const {
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

  // The current block of vertices and faces.
  int total_v = 0;
  int total_f = 0;

  // Add each mesh to be rendered.
  for (const Particle *particle : particles_) {
    int nb_v = particle->getV().rows();
    int nb_f = particle->getF().rows();

    V.block(total_v, 0, nb_v, 3) = particle->getV();
    F.block(total_f, 0, nb_f, 3) = particle->getF().array() + total_v;

    // Get the next offset.
    total_v += nb_v;
    total_f += nb_f;
  }
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
      double j = -(1.0 + c) * v_minus / (1.0 / m1 + 1.0 / m2);
      Vector3d J = j * n_hat;

      // Apply impulse.
      particles_[a]->v += 1.0 / m1 * J;
      particles_[b]->v -= 1.0 / m2 * J;
    }
  }
}

VectorXd Simulation::getForces() const {
  // Accumulation of different forces.
  VectorXd force(particles_.size() * 3);
  force.setZero();

  getGravityForce(force);
  getBoundaryForce(force);
  // getPressureForce(force);
  // getViscosityForce(force);

  return force;
}

void Simulation::getGravityForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++)
    force(i * 3 + 1) += particles_[i]->m * params->gravity;
}

void Simulation::getBoundaryForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    const Vector3d &c = particles_[i]->c;
    double m = particles_[i]->m;

    for (int j = 0; j < 3; j++) {
      double p = c(j);
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

void Simulation::getPressureForce(VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    Vector3d f_i(0.0, 0.0, 0.0);

    for (int j = 0; j < particles_.size(); j++) {
      double m = particles_[j]->m;
      double p_i = particles_[i]->getPressure();
      double p_j = particles_[j]->getPressure();
      double rho_j = particles_[j]->rho;

      Vector3d u = particles_[i]->c - particles_[j]->c;
      double r = u.norm();
      double s = params->kernel_support;
      double c = r / s;

      // Gradient of kernel.
      double dwdr = 1.0 / (M_PI * pow(s, 4));

      if (0.0 <= c && c <= 1.0)
        dwdr *= (3.0 * c) * (-1.0 + 3.0 / 4.0 * c);
      else if (1.0 <= c && c <= 2.0)
        dwdr *= (-3.0 / 4.0 * pow(2.0 - c, 2));
      else
        dwdr *= 0.0;

      f_i += m * (p_i + p_j) / 2.0 / rho_j * dwdr * u;
    }

    force.segment<3>(i * 3) -= f_i;
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
      double s = params->kernel_support;
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
