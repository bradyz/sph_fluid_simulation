git clone --recursive https://github.com/bradyz/sph_fluid_simulation.git
mkdir build
cd build
cmake ..
make -j4 && ./sph
