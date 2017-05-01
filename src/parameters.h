#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "mesh.h"

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

  bool show_surface;
  double surface;
  int resolution;

  Mesh *sphere_mesh;

  Parameters() {
    sphere_mesh = new Mesh("../obj/sphere.obj", 10.0);
    reset();
  }

  void reset();

  ~Parameters() { delete sphere_mesh; }

};

#endif
