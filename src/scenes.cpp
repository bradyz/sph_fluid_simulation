#include "scenes.h"
#include "parameters.h"
#include "particle.h"
#include "bounding_box.h"

#include <vector>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

// namespace for scene helpers.
namespace {

void waterDrop(Parameters *params, vector<Particle*> &particles, int n,
               const Vector3d &translation) {
  double offset = (3.0 * n * params->radius) / 2.0;
  Vector3d center(offset, offset, offset);

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < n; k++) {
        Particle *particle = new Particle(params->sphere_mesh);

        particle->m = params->mass;
        particle->r = params->radius;
        particle->c = Vector3d(3.0 * i * particle->r,
                               3.0 * j * particle->r,
                               3.0 * k * particle->r);
        particle->c += translation - center;

        particle->v = Vector3d(0.0, 0.0, 0.0);
        particle->k = params->gas_constant;
        particle->rho_0 = params->density;
        particle->rho = params->density;
        particle->mu = params->viscocity;

        particle->interface_color = -0.5;
        particle->surface_color = 1.0;

        particles.push_back(particle);
      }
    }
  }
}

void fillBox(Parameters *params, vector<Particle*> &particles,
             const Vector3d &lower, const Vector3d &upper) {
  double offset = 2.4 * params->radius;

  for (double x = lower(0); x <= upper(0); x += offset) {
    for (double y = lower(1); y <= upper(1); y += offset) {
      for (double z = lower(2); z <= upper(2); z += offset) {
        Particle *particle = new Particle(params->sphere_mesh);

        particle->m = params->mass;
        particle->r = params->radius;
        particle->c = Vector3d(x, y, z);

        particle->v = Vector3d(0.0, 0.0, 0.0);
        particle->k = params->gas_constant;
        particle->rho_0 = params->density;
        particle->rho = params->density;
        particle->mu = params->viscocity;

        particle->interface_color = -0.5;
        particle->surface_color = 1.0;

        particles.push_back(particle);
      }
    }
  }
}

// Order is h, w to allow for default params.
BoundingBox *verticalPlane(double x1, double y1, double x2, double y2,
                           double h, double w=0.15) {
  return new BoundingBox(Vector3d(x1-w, -w, y1-w), Vector3d(x2+w, h, y2+w));
}

BoundingBox *horizontalPlane(double x1, double y1, double x2, double y2,
                             double h=0.15) {
  return new BoundingBox(Vector3d(x1, -h, y1), Vector3d(x2, 0.0, y2));
}

} // end namespace for scene helpers.

void Scenes::dropOnPlane(Parameters *params,
                         vector<Particle*> &particles,
                         vector<BoundingBox*> &bounds) {
  particles.clear();
  bounds.clear();

  params->time_step = 0.075;
  params->viscocity = 2.0;

  double b = 2.5;

  waterDrop(params, particles, params->nb_particles, Vector3d(0.0, 3.0, 0.0));
  waterDrop(params, particles, params->nb_particles, Vector3d(1.0, 5.0, 0.0));
  waterDrop(params, particles, params->nb_particles, Vector3d(0.0, 7.0, 1.0));

  bounds.push_back(horizontalPlane(-b, -b, b, b));
}

void Scenes::dropBunny(Parameters *params,
                       vector<Particle*> &particles,
                       vector<BoundingBox*> &bounds) {
  params->viscocity = 1.0;
  params->time_step = 0.1;

  Mesh bunny("../obj/bunny.obj", 25.0);

  const Eigen::MatrixX3d &V_bunny = bunny.getV();

  for (int i = 0; i < V_bunny.rows(); i++) {
    Particle *particle = new Particle(params->sphere_mesh);

    particle->m = params->mass;
    particle->r = params->radius;
    particle->c = V_bunny.row(i);
    particle->c += Vector3d(0.0, 1.5, 0.0);

    particle->v = Vector3d(0.0, 0.0, 0.0);
    particle->k = params->gas_constant;
    particle->rho_0 = params->density;
    particle->rho = params->density;
    particle->mu = params->viscocity;

    particle->interface_color = -0.5;
    particle->surface_color = 1.0;

    particles.push_back(particle);
  }

  double b = 2.5;

  bounds.push_back(horizontalPlane(-b, -b, b, b));
}

void Scenes::damOpening(Parameters *params,
                        vector<Particle*> &particles,
                        vector<BoundingBox*> &bounds) {
  particles.clear();
  bounds.clear();

  params->time_step = 0.075;

  // Water.
  params->viscocity = 0.1;

  double w = 0.5;
  double l = 3.0;
  double h = 5.0;

  // Floor.
  bounds.push_back(horizontalPlane(-l, -w, l, w));

  // Walls.
  bounds.push_back(verticalPlane(-l, -w, -l,  w, h));
  bounds.push_back(verticalPlane(-l, -w,  l, -w, h));
  bounds.push_back(verticalPlane( l, -w,  l,  w, h));
  bounds.push_back(verticalPlane(-l,  w,  l,  w, h));

  fillBox(params, particles,
          Vector3d(-l + 0.2,     0.1, -w + 0.2),
          Vector3d(    -0.5, 0.3 * h,  w - 0.2));
}
