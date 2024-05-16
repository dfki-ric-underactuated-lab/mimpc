# Installation
The full blueprint to install the lib and the examples is also available in the (Dockerfile).
## Dependencies
### Library
#### SCIP Solver
Refer to the instructions in https://scipopt.org/index.php#download.
There are precompiled packages available.
Otherwise, install from sources.
#### EigenLib
Eigen is a header lib, available at https://eigen.tuxfamily.org/.
For Ubuntu there is also a package available:
```bash
sudo apt install libeigen3-dev
```
#### LibFMT
Follow instructions in https://fmt.dev/latest/usage.html#building-the-library and https://fmt.dev/latest/usage.html#installing-the-library

### Simulator and Example
#### Drake
Follow instructions in https://drake.mit.edu/installation.html
#### Cnpy
Follow the instructions in https://github.com/rogersce/cnpy
#### ZLIB
Install ZLIB available on https://www.zlib.net/.
For Ubuntu there is also a package available:
```bash
sudo apt install zlib1g-dev
```

## Build and install
```bash
mkdir build
cmake -DBuildSim=ON -DBuildExamples=ON ..
make
sudo make install
```
The cmake flags ```BuildSim``` and ```BuildExamples``` control weather the free floating satellite simulation and the example project based on Drake should be installed as well.
