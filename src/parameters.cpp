#include "parameters.h"

void Parameters::reset() {
  scene_mode = SceneMode::SLOSH;

  paused = true;
  time_step = 0.05;

  gravity = -9.8;

  coefficient_of_restitution = 0.8;
  penalty_coefficient = 1e6;

  nb_particles = 7;

  radius = 0.06;
  mass = 0.1;
  density = 20.0;
  viscocity = 0.50;
  gas_constant = 20.0;

  kernel_radius = 0.25;

  view_mode = ViewMode::DENSITY;

  surface = 1.0;
  resolution = 30;

  fps_cap = 15.0;

  fluid_velocity_max = 1.0;
  jet_min = 2.8;
  jet_max = 3.8;
}
