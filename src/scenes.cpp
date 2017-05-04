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

        particles.push_back(particle);
      }
    }
  }
}

// Order is h, w to allow for default params.
BoundingBox *verticalPlane(double x1, double y1, double x2, double y2,
                           double h, double w=0.25) {
  return new BoundingBox(Vector3d(x1-w, -w, y1-w), Vector3d(x2+w, h, y2+w));
}

BoundingBox *horizontalPlane(double x1, double y1, double x2, double y2,
                             double h=0.25) {
  return new BoundingBox(Vector3d(x1, -h, y1), Vector3d(x2, 0.0, y2));
}

} // end namespace for scene helpers.

void Scenes::dropOnPlane(Parameters *params,
                         vector<Particle*> &particles,
                         vector<BoundingBox*> &bounds) {
  particles.clear();
  bounds.clear();

  waterDrop(params, particles, params->nb_particles, Vector3d(0.0, 3.0, 0.0));
  waterDrop(params, particles, params->nb_particles, Vector3d(1.0, 5.0, 0.0));
  waterDrop(params, particles, params->nb_particles, Vector3d(0.0, 7.0, 1.0));

  BoundingBox *box;

  box = new BoundingBox(Vector3d(-4.0, -0.5, -4.0), Vector3d(4.0, 0.0, 4.0));
  bounds.push_back(box);
}

void Scenes::damOpening(Parameters *params,
                        vector<Particle*> &particles,
                        vector<BoundingBox*> &bounds) {
  particles.clear();
  bounds.clear();

  waterDrop(params, particles, params->nb_particles, Vector3d(0.0, 7.0, 0.0));

  double b = 2.0;

  bounds.push_back(horizontalPlane(-b, -b, b, b));
  bounds.push_back(verticalPlane(-b, -b, -b, b, 4.0));
  bounds.push_back(verticalPlane(-b, -b, b, -b, 4.0));
  bounds.push_back(verticalPlane(b, -b, b, b, 4.0));
  bounds.push_back(verticalPlane(-b, b, b, b, 4.0));
}
