#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "mesh.h"

enum ViewMode {
  DENSITY,
  SURFACE,
  VELOCITY,
};

enum SceneMode {
  DROP,
  BUNNY,
  DAM,
};

class Parameters {

public:
  SceneMode scene_mode;

  bool paused;                                      // animation on.
  double time_step;                                 // s? hopefully.

  double gravity;

  double coefficient_of_restitution;
  double penalty_coefficient;

  int nb_particles;                                 // number particles.
  double radius;                                    // render radius.
  double mass;                                      // kg.
  double density;                                   // kg / m^3.
  double viscocity;                                 // N s / m^3.
  double gas_constant;                              // N m / kg.
  double interface_sigma;                           // Scaling of interface force
  double surface_sigma;                             // Scaling of surface force

  double kernel_radius;

  ViewMode view_mode;
  bool show_points;

  double surface;
  int resolution;

  double fps_cap;

  double fluid_velocity_min;
  double fluid_velocity_max;
  double jet_min;
  double jet_max;

  Mesh *sphere_mesh;

  Parameters() {
    sphere_mesh = new Mesh("../obj/sphere.obj", 10.0);
    reset();
  }

  void reset();

  ~Parameters() { delete sphere_mesh; }

};

#endif
