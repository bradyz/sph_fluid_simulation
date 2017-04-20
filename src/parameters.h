#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters {

public:
  bool paused;                                      // animation on.
  double time_step;                                 // s? hopefully.

  double gravity;

  double coefficient_of_restitution;
  double penalty_coefficient;
  double boundary_min;
  double boundary_max;

  int nb_particles;                                 // number particles.
  double radius;                                    // render radius.
  double mass;                                      // kg.
  double density;                                   // kg / m^3.
  double viscocity;                                 // N s / m^3.
  double gas_constant;                              // N m / kg.

  double kernel_radius;

  Parameters() {
    reset();
  }

  void reset() {
    paused = true;
    time_step = 0.006;

    gravity = -9.8;

    coefficient_of_restitution = 0.2;
    penalty_coefficient = 1e3;
    boundary_min = 0.0;
    boundary_max = 5.0;

    nb_particles = 5;

    radius = 0.06;
    mass = 0.012;
    density = 1000.0;
    viscocity = 50.0;
    gas_constant = 20.0;

    kernel_radius = 0.045;
  }

};

#endif
