.. _dependencies:

System Requirements
-------------------

The software is written in Python 3.10 and tested on MacOs and Linux. The usage of the Anaconda_ Python distribution
is strongly recommended. The requirements are a C++17 compiler and CMake. The requirements for the Python wrapper
are C++17 compiler, CMake, and at least Python 3.10 with development headers. The following minimum versions of CMake and pip
are required:

* **CMake**: version 3.20 or above
* **Pip**: version 21.3 or above

If you are a Mac user, we additionally recommend you to use Homebrew_, allowing you to install required dependencies such as Eigen.

.. _Anaconda: http://www.anaconda.com/download/#download
.. _Homebrew: https://brew.sh


Essential Third Party Libraries and Packages
--------------------------------------------

The CommonRoad Drivability Checker depends on following third party libraries and Python packages:

Third-party libraries

* `Box2D <https://github.com/erincatto/box2d>`_
* `FCL -- The Flexible Collision Library <https://github.com/flexible-collision-library/fcl>`_
* `libccd <https://github.com/danfis/libccd>`_
* `Eigen3 <https://eigen.tuxfamily.org/dox/>`_ 
* `Boost <https://www.boost.org/>`_
* `pybind11 <https://github.com/pybind/pybind11>`_
* `OpenMP <https://www.openmp.org/>`_
* `Pandoc <https://pandoc.org>`__ (for the documentation)
* `Doxygen <http://www.doxygen.nl>`_ (for the documentation)

Python packages (see also `pyproject.toml`)

* `commonroad-io <https://pypi.org/project/commonroad-io/>`_
* `commonroad-vehicle-models <https://pypi.org/project/commonroad-vehicle-models/>`_
* `numpy <https://pypi.org/project/numpy/>`_
* `Shapely <https://pypi.org/project/Shapely/>`_
* `matplotlib <https://pypi.org/project/matplotlib/>`_
* `Jupyter <https://pypi.org/project/jupyter/>`_ (for the tutorials)
* `Triangle <https://pypi.org/project/triangle/>`_
* `scipy <https://pypi.org/project/scipy/>`_
* `sphinx_rtd_theme <https://pypi.org/project/sphinx-rtd-theme/>`_
* `sphinx <https://pypi.org/project/Sphinx/>`_
* `nbsphinx_link <https://pypi.org/project/nbsphinx-link/>`_
* `nbsphinx <https://pypi.org/project/nbsphinx/>`_
* `breathe <https://pypi.org/project/breathe/>`_
* `polygon3 <https://pypi.org/project/Polygon/>`_


Optional Third Party Libraries
------------------------------

One can optionally modify our C++ library to use a plugin for fast triangulation of polygons which uses the CGAL library:

* `CGAL <https://github.com/CGAL/cgal>`_ 

For serialization, we use the s11n libary:

* `s11n <http://www.s11n.net/>`_


