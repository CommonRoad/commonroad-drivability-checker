CommonRoad Drivability Checker
------------------------------

Collision avoidance, kinematic feasibility, and road-compliance must be
validated to ensure the drivability of planned motions for autonomous
vehicles. The CommonRoad Drivability Checker toolbox unifies these checks
in order to simplify the development and validation of motion planning
algorithms. It is compatible with the CommonRoad benchmark suite, which
additionally facilitates and drastically reduces the effort of the development
of motion planning algorithms.

Please post questions, bug reports, etc. related to our tools or website in our [forum](https://commonroad.in.tum.de/forum/).


Installation
------------

We provide two installation options: Installation as a Python package or building from source.

1. **Python Package**: Install the python package via `pip` in your Conda environment:

	```bash
	pip install commonroad-drivability-checker
	```
	**Note for MacOS M1 users**: You need to use the 64-bit Anaconda Installer (graphical or command-line) in order to install the MacOS PyPi package.

2. **Build from source**: To build the drivability checker from source, please refer to the installation description in the 
[documentation](https://cps.pages.gitlab.lrz.de/commonroad-drivability-checker/).


System Requirements
-------------------
The software is written in Python 3.7 and C++11 and tested on MacOS and Linux. 

The usage of the **[Anaconda](http://www.anaconda.com/download/#download9) Python distribution** is strongly recommended. 

For building the code from source, the following minimum versions are required:
  * **GCC and G++**: version 9 or above
  * **CMake**: version 3.10 or above.
  * **Pip**: version 21.3 or above

**Note for MacOS users (M1 or Intel):** we additionally recommend using the Homebrew package manager, to install required dependencies such as Eigen.


Third Party Libraries and Packages
----------------------------------
The following third-party dependencies of the C++ code are only required for building the project from source!

**Essential dependencies**:

Not included as submodules:
* [Eigen3](https://eigen.tuxfamily.org/dox/)
* [Boost](https://www.boost.org/)
* [OpenMP](https://www.openmp.org/)

Included as submodules:
* [Box2D](https://github.com/erincatto/box2d)
* [FCL](https://github.com/flexible-collision-library/fcl)
* [libccd](https://github.com/danfis/libccd)
* [gpc](https://github.com/rickbrew/GeneralPolygonClipper)
* [pybind11](https://github.com/pybind/pybind11)
* [libs11n](http://www.s11n.net/)

**Optional dependencies**:
* [Triangle](https://www.cs.cmu.edu/~quake/triangle.html) (optional library for triangulation, not built by default)
* [CGAL](https://github.com/CGAL/cgal) (optional library for triangulation, not built by default)
* [Pandoc](https://pandoc.org) (optional for building documentation)
* [Doxygen](http://www.doxygen.nl) (optional for building documentation)

**Note**: Please be aware of the specific licensing conditions when including the optional dependencies Triangle and CGAL.
See also `notes.txt` and the licensing information on the respective package websites for more details.

The Python dependencies are listed in `requirements.txt`.


Documentation
-------------
A full documentation as well as tutorials to get started with the tool can be found on our [toolpage](https://commonroad.in.tum.de/tools/drivability-checker).


Publication
-----------
#### CommonRoad Drivability Checker: Simplifying the Development and Validation of Motion Planning Algorithms
Christian Pek, Vitaliy Rusinov, Stefanie Manzinger, Murat Can Üste, and Matthias Althoff

Abstract— Collision avoidance, kinematic feasibility, and road-compliance must be validated to ensure the drivability
of planned motions for autonomous vehicles. Although these tasks are highly repetitive, computationally efficient
toolboxes are still unavailable. The CommonRoad Drivability Checker—an open-source toolbox—unifies these mentioned
checks. It is compatible with the CommonRoad benchmark suite, which additionally facilitates the development of motion
planners. Our toolbox drastically reduces the effort of developing and validating motion planning algorithms. Numerical
experiments show that our toolbox is real-time capable and can be used in real test vehicles.

Fulltext is available on [mediaTUM](https://mediatum.ub.tum.de/doc/1546126/).

##### Bibtex:
```
@inproceedings{ PekIV20.pdf,
	author = "Christian Pek, Vitaliy Rusinov, Stefanie Manzinger, Murat Can Üste, and Matthias Althoff",
	title = "CommonRoad Drivability Checker: Simplifying the Development and Validation of Motion Planning Algorithms",
	pages = "1-8",
	booktitle = "Proc. of the IEEE Intelligent Vehicles Symposium",
	year = "2020",
	abstract = "Collision avoidance, kinematic feasibility, and road-compliance must be validated to ensure the drivability of planned motions for autonomous vehicles. Although these tasks are highly repetitive, computationally efficient toolboxes are still unavailable. The CommonRoad Drivability Checker— an open-source toolbox—unifies these mentioned checks. It is compatible with the CommonRoad benchmark suite, which additionally facilitates the development of motion planners. Our toolbox drastically reduces the effort of developing and validating motion planning algorithms. Numerical experiments show that our toolbox is real-time capable and can be used in real test vehicles."
}
```
