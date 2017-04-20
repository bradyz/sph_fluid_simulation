##Info

Team Members:
Arnav Sastry
Brady Zhou

This is a SPH fluid simulation, the setup simulation drops a cube of viscous fluid onto the floor.

Features:
Rigid body collisions
Penalty force for floor
Pressure force
Viscosity force
Cube spline kernel for smoothing (estimating laplacian over attribute field)
The color of each particle is the density.
UI is nanogui (provided by LibIGL)

##Installation

Do the following -

```
cd sph_fluid_simulation

mkdir build

cd build

cmake ..

make -j4 && ./sph
```

##Tips

GUI hotkeys - "r" to reset simulation, "space" to to start/stop simulation.
