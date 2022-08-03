.. _dependencies:

System Requirements
-------------------

The software is written in Python 3.6/3.7 and tested on MacOs and Linux. The usage of the Anaconda_ Python distribution
is strongly recommended. The requirements are a C++11 compiler and CMake. The requirements for the Python wrapper
are C++11 compiler, CMake, and min. Python 3.6 with development headers. The following minimum versions of CMake and pip
are required:

* **CMake**: version 3.10 or above
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
* `OpenMP <https://www.openmp.org/>`_ (for the geometry submodule)
* `Pandoc <https://pandoc.org>`__ (for the documentation)
* `Doxygen <http://www.doxygen.nl>`_ (for the documentation)

Python packages (see also requirements file)

* `commonroad-io <https://pypi.org/project/commonroad-io/>`_ (>=2020.2)
* `commonroad-vehicle-models <https://pypi.org/project/commonroad-vehicle-models/>`_ (>=1.0.0)
* `numpy <https://pypi.org/project/numpy/>`_ (>=1.19)
* `Shapely <https://pypi.org/project/Shapely/>`_ (>=1.6.4)
* `matplotlib <https://pypi.org/project/matplotlib/>`_ (>=3.2.2)
* `Jupyter <https://pypi.org/project/jupyter/>`_ (>=1.0.0, for the tutorials)
* `Triangle <https://pypi.org/project/triangle/>`_ (>=20200424)
* `scipy <https://pypi.org/project/scipy/>`_ (>=1.4.1)
* `pandoc <https://pypi.org/project/pandoc/>`__ (>=1.0.2)
* `sphinx_rtd_theme <https://pypi.org/project/sphinx-rtd-theme/>`_ (>=0.4.3)
* `sphinx <https://pypi.org/project/Sphinx/>`_ (>=3.0.3)
* `nbsphinx_link <https://pypi.org/project/nbsphinx-link/>`_ (>=1.3.0)
* `nbsphinx <https://pypi.org/project/nbsphinx/>`_ (>=0.6.1)
* `breathe <https://pypi.org/project/breathe/>`_ (>=4.18.0)
* `polygon3 <https://pypi.org/project/Polygon/>`_ (>=3.0.8)


Optional Third Party Libraries
------------------------------

One can optionally modify our C++ library to use a plugin for fast triangulation of polygons which uses the CGAL library:

* `CGAL <https://github.com/CGAL/cgal>`_ 

For serialization, we use the s11n libary:

* `s11n <http://www.s11n.net/>`_


