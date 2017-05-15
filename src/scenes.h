#ifndef SCENES_H
#define SCENES_H

#include "parameters.h"
#include "particle.h"
#include "bounding_box.h"

#include <vector>

// begin namespace Scenes.
namespace Scenes {

void dropOnPlane(Parameters *params,
                 std::vector<Particle*> &particles,
                 std::vector<BoundingBox*> &bounds);

void dropBunny(Parameters *params,
               std::vector<Particle*> &particles,
               std::vector<BoundingBox*> &bounds);

void damOpening(Parameters *params,
                std::vector<Particle*> &particles,
                std::vector<BoundingBox*> &bounds);

void twoLiquids(Parameters *params,
                std::vector<Particle*> &particles,
                std::vector<BoundingBox*> &bounds);

} // end namespace Scenes.

#endif
