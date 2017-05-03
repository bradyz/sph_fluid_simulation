#include "parameters.h"

#include <cmath>

void Parameters::reset() {
  paused = true;
  time_step = 0.05;

  gravity = -9.8;

  coefficient_of_restitution = 0.2;
  penalty_coefficient = 1e4;
  boundary_min = 0.0;
  boundary_max = 5.0;

  nb_particles = 7;

  radius = 0.06;
  mass = 0.112;
  density = 10.0;
  viscocity = 0.5;
  gas_constant = 20.0;

  kernel_radius = 0.500;

  show_surface = false;
  surface = 1.0;
  resolution = 30;

  fps_cap = 20.0;
}
