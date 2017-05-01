#ifndef SCENES_H
#define SCENES_H

#include "parameters.h"
#include "particle.h"

#include <vector>

// begin namespace Scenes.
namespace Scenes {

std::vector<Particle*> dropOnPlane(Parameters *params);

} // end namespace Scenes.

#endif
