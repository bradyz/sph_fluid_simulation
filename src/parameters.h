#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters {

public:
  bool paused;                                      // animation on.
  double time_step;                                 // s? hopefully.

  double gravity;

  double penalty_coefficient;
  double boundary_min;
  double boundary_max;

  int nb_particles;                                 // number particles.
  double radius;                                    // render radius.
  double mass;                                      // kg.
  double density;                                   // kg / m^3.
  double viscocity;                                 // N s / m^3.
  double gas_constant;                              // N m / kg.

  double kernel_support;

  Parameters() {
    reset();
  }

  void reset() {
    paused = true;
    time_step = 0.001;

    gravity = -9.8;

    penalty_coefficient = 1e6;
    boundary_min = 0.0;
    boundary_max = 1.0;

    nb_particles = 10;

    radius = 0.01;
    mass = 0.012;
    density = 1000.0;
    viscocity = 50.0;
    gas_constant = 20.0;

    kernel_support = 50.0;
  }

};

#endif
