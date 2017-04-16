#include "simulation.h"
#include "parameters.h"

#include <functional>

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

  meshes_.push_back(new Mesh("../obj/sphere.obj"));

  for (int i = 0; i < params->nb_particles; i++) {
    for (int j = 0; j < params->nb_particles; j++) {
      for (int k = 0; k < params->nb_particles; k++) {
        Particle *particle = new Particle(meshes_.front());
        particle->m = params->mass;
        particle->r = params->radius;
        particle->c = Vector3d(i * 0.1, j * 0.1, k * 0.1);
        particle->v = Vector3d(i * 0.1, j * 0.1, k * 0.1);

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

  // New positions.
  VectorXd c_new(n * 3);

  // Velocity Verlet (update position, then velocity).
  for (int i = 0; i < n; i++) {
    c_new.segment<3>(i * 3) = particles_[i]->c + h * particles_[i]->v;

    particles_[i]->c = c_new.segment<3>(i * 3);
  }

  // F = -dV'.
  VectorXd forces = getForces(c_new);

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

VectorXd Simulation::getForces(const VectorXd &c_new) const {
  // Accumulation of different forces.
  VectorXd force(c_new.rows());
  force.setZero();

  getGravityForce(c_new, force);
  getBoundaryForce(c_new, force);

  return force;
}

void Simulation::getGravityForce(const VectorXd &c_new, VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++)
    force(i * 3 + 1) += particles_[i]->m * params->gravity;
}

void Simulation::getBoundaryForce(const VectorXd &c_new, VectorXd &force) const {
  for (int i = 0; i < particles_.size(); i++) {
    const Vector3d &c = c_new.segment<3>(i * 3);
    double m = particles_[i]->m;

    for (int j = 0; j < 3; j++) {
      double p = c(j);
      double k = params->boundary_penalty;
      double b_min = params->boundary_min;
      double b_max = params->boundary_max;

      if (p < b_min)
        force(i * 3 + j) += (b_min - p) * k * m;
      if (p > b_max)
        force(i * 3 + j) += (b_max - p) * k * m;
    }
  }
}

Simulation::~Simulation() {
  delete params;

  for (Particle *particle : particles_)
    delete particle;
  for (Mesh *mesh : meshes_)
    delete mesh;
}
