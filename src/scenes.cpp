#include "scenes.h"
#include "parameters.h"
#include "particle.h"
#include "bounding_box.h"

#include <vector>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

void Scenes::dropOnPlane(Parameters *params,
                         vector<Particle*> &particles,
                         vector<BoundingBox*> &bounds) {
  particles.clear();
  bounds.clear();

  for (int i = 0; i < params->nb_particles; i++) {
    for (int j = 0; j < params->nb_particles; j++) {
      for (int k = 0; k < params->nb_particles; k++) {
        Particle *particle = new Particle(params->sphere_mesh);

        particle->m = params->mass;
        particle->r = params->radius;
        particle->c = Vector3d(3.0 * i * particle->r,
                               3.0 * j * particle->r,
                               3.0 * k * particle->r);

        particle->v = Vector3d(0.0, 0.0, 0.0);
        particle->k = params->gas_constant;
        particle->rho_0 = params->density;
        particle->rho = params->density;
        particle->mu = params->viscocity;

        particles.push_back(particle);
      }
    }
  }

  BoundingBox *box;

  box = new BoundingBox(Vector3d(-4.0, -0.5, -4.0), Vector3d(4.0, 0.0, 4.0));
  bounds.push_back(box);
}
