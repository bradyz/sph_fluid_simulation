#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters {

public:
  bool paused;                                      // animation on.
  int nb_particles;                                 // number particles.
  double radius;                                    // render radius.
  double mass;                                      // kg.
  double density;                                   // kg / m^3.
  double viscocity;                                 // N s / m^3.

  Parameters() {
    reset();
  }

  void reset() {
    paused = true;

    nb_particles = 10;

    radius = 0.1;
    mass = 0.012;
    density = 1000.0;
    viscocity = 50.0;
  }

};

#endif
