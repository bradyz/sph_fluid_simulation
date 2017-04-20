#include "parameters.h"

void Parameters::reset() {
  paused = true;
  time_step = 0.006;

  gravity = -9.8;

  coefficient_of_restitution = 0.2;
  penalty_coefficient = 1e4;
  boundary_min = 0.0;
  boundary_max = 5.0;

  nb_particles = 5;

  radius = 0.06;
  mass = 0.112;
  density = 1.225;
  viscocity = 50.0;
  gas_constant = 20.0;

  kernel_radius = 0.500;
}
