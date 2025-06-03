.. _installation:

Installation of the Python interface
------------------------------------

**Note:**
The following steps are necessary if you intend to use the CommonRoad Drivability
Checker from Python. They are *not* necessary if you only intend to use
the CommonRoad Drivability Checker from C++.

In order to install CommonRoad Drivability Checker, we first need to install essential third party libraries,
followed by installation of optional third party libraries, and finally the installation of CommonRoad Drivability
Checker itself. All required steps are described in the following sections.

Before installing anything, we recommend you to install Anaconda_ and to create an Anaconda environment.
Below, we assume that your Anaconda environment is called **commonroad-py36**. In the root folder of the
CommonRoad Drivability Checker, activate your environment with:

  .. code-block:: bash

      $ conda activate commonroad-py36

If you choose to continue with python virtualenv please refer to the official documentation_, and create an
environment with the appropriate Python version, and activate your environment.

.. _Anaconda: https://www.anaconda.com/download/#download
.. _documentation: https://docs.python.org/3/tutorial/venv.html

Installation of Essential Third Party Libraries
***********************************************

The following dependencies need to be installed manually:

* `OpenMP <https://www.openmp.org/>`_
* `Pandoc <https://pandoc.org>`__ (only required for building the documentation)
* `Doxygen <http://www.doxygen.nl>`_ (only required for building the documentation)

Installation on Linux (e.g. using apt-get for Debian derivatives):

.. code-block:: bash

    $ sudo apt-get install libomp-dev doxygen pandoc

Installation on macOS using `Homebrew <https://brew.sh/>`_:

.. code-block:: bash

    $ brew install doxygen pandoc

On macOS, the OpenMP library needs to be installed manually.
The version of the OpenMP library must correspond to the version of the Apple C++ compiler currently installed on your Mac (g++ --version).
You can download the corresponding version of the library from https://mac.r-project.org/openmp/ and follow the installation instructions.


The following third party libraries will automatically be downloaded and compiled by the build system:

* `Boost <https://www.boost.org/>`_
* `Eigen <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_
* `Box2D <https://github.com/erincatto/box2d>`_
* `FCL -- The Flexible Collision Library <https://github.com/flexible-collision-library/fcl>`_
* `libccd <https://github.com/danfis/libccd>`_
* `Triangle <https://pypi.org/project/triangle/>`_ (for the C++ library)
* `pybind11 <https://github.com/pybind/pybind11>`_

To speed up the build, you may also install these libraries manually.

Installation of the CommonRoad Drivability Checker
**************************************************

After installing all essential third party libraries, you can now install the CommonRoad Drivability Checker.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your conda environment with

    .. code-block:: bash

            $ conda activate commonroad-py36

#. Please make sure that the latest pip version is installed. Otherwise, the next installation step would not work.

    .. code-block:: bash

            $ python -m pip install --upgrade pip

#. Compile and install the CommonRoad Drivability Checker library by running

        .. code-block:: bash

            $ pip install -v .


Installation of Optional Third Party Libraries
**********************************************

(Recommended) Some optional functions for polygon triangulation require a third-party library Triangle to be installed. To do so, run

  .. code-block:: bash

      $ pip install --config-settings=cmake.args=-DCR_DC_USE_TRIANGLE=ON -v .[triangle]
      
Please note that although Triangle is freely available, it is copyrighted by its author and may 
not be sold or included in commercial products without a license. See the original `licensing conditions <https://github.com/wo80/Triangle/blob/master/src/Triangle/README>`_ 
of the Triangle library as well as the `Triangle website <https://www.cs.cmu.edu/~quake/triangle.html>`_ for details.

It is possible to modify Drivability Checker to support CGAL. For the installation of CGAL, please refer to `their website <https://github.com/CGAL/cgal>`_.
