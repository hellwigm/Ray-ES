# Ray-ES

This is the code Ray-ES of the supplementary material for the
paper "A Novel and Simple Approach for Constrained Optimization -
An Evolution Strategy that Evolves Rays" by Patrick Spettel and
Hans-Georg Beyer.

## License
Ray-ES is licensed under the GNU General Public License.

Ray-ES is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Ray-ES is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Ray-ES.  If not, see <http://www.gnu.org/licenses/>.

## Dependencies
The code depends on the following third-party libraries:

* [Eigen](http://eigen.tuxfamily.org)

## Build and installation instructions
[CMake](cmake.org) is used as the build system.
The third-party libraries are located by using CMake's `find_package`
command. The user is responsible for installing the third-party dependencies
and providing the appropriate paths to CMake.

Build and installation can be done with these commands
if all third-party libraries are installed and can be found by CMake:

    $ cd /path/to/Ray-ES
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_INSTALL_PREFIX=<install prefix> \
            -DCMAKE_BUILD_TYPE=<build type> \
      ..
    $ make
    $ make install

Additionally, instead of typing the above commands,
two Bash helper scripts are provided. They can be
used as well. One is for configuring (`config_cmake.sh`)
and for building (`make.sh`). They use `./build` as
the build directory and `./install` as the installation
directory.

    $ cd /path/to/Ray-ES
    $ ./config_cmake.sh
    $ ./make.sh

We have only built and run this tool on a GNU/Linux system
with GCC. But you should be able to easily build it on any
system supporting CMake and a C++14 compiler.

## Running
A small usage example is provided in `rayes/src/main.cpp`.
It creates a 2-dimensional sphere optimization problem with one linear
constraint and applies the Ray-ES to this problem.
It is built as an executable named `es_rayestool` using CMake.
And it is installed to `<install prefix>/bin/es_rayestool`.
After building, it can be run with the following command:

    $ <install prefix>/bin/es_rayestool

The expected output is something like this:

    Termination criterion: SigmaLimitReached.
    abs(fBest - (f of best point)) / fBest = 3.43476e-13

## Running in the BBOB COCO framework
In order to run the Ray-ES in the BBOB COCO framework first get and build
the BBOB COCO framework for C/C++. Note that we tested the Ray-ES for both
linear and non-linear constraints. The original BBOB COCO framework
applies non-linear perturbations to the problems in the `bbob-constrained`
suite. We therefore provide
an adapted version of the BBOB COCO framework in a GitHub fork.
This adapted version has the ability to disable
the non-linear perturbations.
By default the non-linear perturbations are disabled.
They can be enabled using the `#define`s
`ENABLE_NON_LINEAR_TRANSFORMATIONS_ON_CONSTRAINTS`
and
`ENABLE_NON_LINEAR_TRANSFORMATIONS_ON_OBJECTIVEFUNC`.

The needed BBOB COCO files `coco.c` and `coco.h` are already provided
in `/path/to/Ray-ES/coco`. They are provided such that the code can be
run without the need to get the BBOB COCO files. If you want to use another
version of BBOB COCO, you can use the instructions below as a guideline.

The command

    $ git clone https://github.com/patsp/coco.git

clones the repository into a folder called `coco` in the current directory.
Our changes are in a branch called `development-sppa-2`.
Issue

    $ cd coco
    $ git checkout development-sppa-2

to change into the `coco` directory and checkout those files.
Build the BBOB COCO framework for C/C++ (see the BBOB COCO
build instructions for all the details).
The command

    $ python do.py build-c

does this and the built files are then in `code-experiments/build/c`.
Copy the files `coco.c` and `coco.h` to `/path/to/Ray-ES-cpp/coco`
and overwrite the existing files.
The command

    $ <install prefix>/bin/coco

runs the Ray-ES on the `bbob-constrained` suite. Adapt the
`coco_experiment_rayes.cpp` to your needs (see the comments).

___

The following commands can be used to obtain a version of the BBOB COCO
framework from the official repository.

The command

    $ git clone https://github.com/numbbo/coco.git

clones the repository into a folder called `coco` in the current directory.
Currently, the `bbob-constrained` suite is in a branch called `development`.
Issue

    $ cd coco
    $ git development

and then

    $ python do.py build-c

to obtain `coco.c` and `coco.h`.

