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

We provide two installation methods:

* **Method 1: Automatic Installation via Bash Script** (recommended for Ubuntu)
* **Method 2: Manual Installation**

Before installing anything, we recommend you to install Anaconda_ and to create an Anaconda environment.
Below, we assume that your Anaconda environment is called **commonroad-py36**. In the root folder of the
CommonRoad Drivability Checker, activate your environment with:

  .. code-block:: bash

      $ conda activate commonroad-py36

If you choose to continue with python virtualenv please refer to the official documentation_, and create an
environment with python3.6m (dev version), and activate you environment.

.. _Anaconda: https://www.anaconda.com/download/#download
.. _documentation: https://docs.python.org/3/tutorial/venv.html

Method 1: Automatic Installation via Bash Script
################################################

**Note:** The installation script only works on Ubuntu based distributions. If your operating system is not Ubuntu
based, please refer to :ref:`method2_manual`, which is described below.

We have provided a build script for easy installation. You still need to activate your conda environment
(or virtual environment if you like). Full installation option will install third party libraries (suggested).

The **-i** option on the build command will install the drivability checker to the environment specified in the path.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your conda environment with the following command (if not already activated)

    .. code-block:: bash

            $ conda activate commonroad-py36

#. Run the build script

        **Basic installation without CGAL**

        .. code-block:: bash

            $ bash build.sh -j JOB_COUNT -i

        **(SUGGESTED) Full installation:**

        .. code-block:: bash

            $ bash build.sh -j JOB_COUNT --cgal -i

        **Note that you have to replace**
         - *JOB_COUNT*  with the number of jobs you are willing to allocate to cmake, for example *-j 2*.
           Each job (possibly) will use a core, so specify this number according to your system and free cores.

        For additional options, please run **bash build.sh -h** command to view them.

.. _method2_manual:

Method 2: Manual Installation
#############################

Prerequisites
*************

We assume that the following libraries are already installed on your system:

* `Eigen3 <https://eigen.tuxfamily.org/dox/>`_ (version >=3.3.7 - preferably the newest version)
* `Boost libraries <https://www.boost.org/>`_ (version >= 1.58)
* `OpenMP <https://www.openmp.org/>`_ (for the geometry submodule support)

For the documentation, we require the libraries `Pandoc <https://pandoc.org>`__ and `Doxygen <http://www.doxygen.nl>`_.
All aforementioned libraries can be installed on Ubunutu via apt-get.

Installation of Essential Third Party Libraries and Packages
************************************************************

The following dependencies are *not* included in this repository and need to
be installed externally, e.g. by using a package manager:

* `Boost <https://www.boost.org/>`_ (only `Boost.Thread`, `Boost.Test`, `Boost.Filesystem`)
* `Eigen <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_

Installation on Linux (e.g. using apt-get for Debian derivatives):

.. code-block:: bash

    $ sudo apt-get install build-essential cmake git wget unzip libboost-dev libboost-thread-dev libboost-test-dev libboost-filesystem-dev libeigen3-dev libomp-dev

Installation on macOS using `Homebrew <https://brew.sh/>`_:

.. code-block:: bash

    $ brew install cmake eigen boost

On macOS, it is also necessary to install the OpenMP library manually. The version of the OpenMP library must correspond to the version of the Apple C++ compiler currently installed on your Mac (g++ --version). One can download the corresponding version of the library from https://mac.r-project.org/openmp/ and follow the installation instructions.


The following third party libraries are included as submodules:

* `Box2D <https://github.com/erincatto/box2d>`_
* `FCL -- The Flexible Collision Library <https://github.com/flexible-collision-library/fcl>`_
* `libccd <https://github.com/danfis/libccd>`_
* `pybind11 <https://github.com/pybind/pybind11>`_
* `Triangle <https://pypi.org/project/triangle/>`_ (for the C++ library)

In order to initialize the bundled submodules,
run the following commands in the root folder of the CommonRoad Drivability Checker:

.. code-block:: bash

        $ git submodule update --init


Following packages are available via `PyPi <https://pypi.org/>`_:

* `commonroad-io <https://pypi.org/project/commonroad-io/>`_
* `commonroad-vehicle-models <https://pypi.org/project/commonroad-vehicle-models/>`_
* `matplotlib <https://pypi.org/project/matplotlib/>`_
* `Shapely <https://pypi.org/project/Shapely/>`_
* `numpy <https://pypi.org/project/numpy/>`_
* `Jupyter <https://pypi.org/project/jupyter/>`_
* `Scipy <https://pypi.org/project/scipy/>`_
* `Pandoc <https://pypi.org/project/pandoc/>`_
* `Sphinx_rtd_theme <https://pypi.org/project/sphinx-rtd-theme/>`_
* `Sphinx <https://pypi.org/project/Sphinx/>`_
* `nbspinxlink <https://pypi.org/project/nbsphinx-link/>`_
* `nbsphinx <https://pypi.org/project/nbsphinx/>`_
* `breathe <https://pypi.org/project/breathe/>`_
* `polygon3 <https://pypi.org/project/Polygon3/>`_


They can be installed with the following command:

  .. code-block:: bash

      $ pip3 install -r requirements.txt

**N.B.** If you are using Conda, make sure your environment is activated!

Installation of Optional Third Party Libraries
**********************************************

(Recommended) Some optional functions for polygon triangulation require a third-party library Triangle to be installed. To do so, run

  .. code-block:: bash

      $ pip3 install triangle
      
Please note that although Triangle is freely available, it is copyrighted by its author and may 
not be sold or included in commercial products without a license. See the original `licensing conditions <https://github.com/wo80/Triangle/blob/master/src/Triangle/README>`_ 
of the Triangle library as well as the `Triangle website <https://www.cs.cmu.edu/~quake/triangle.html>`_ for details.

It is possible to modify Drivability Checker to support CGAL. For the installation of CGAL, please refer to `their website <https://github.com/CGAL/cgal>`_.

Installation of the CommonRoad Drivability Checker
**************************************************

After installing all essential third party libraries and packages, you can now install the CommonRoad Drivability Checker.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your conda environment with

    .. code-block:: bash

            $ conda activate commonroad-py36
            
#. Please make sure that the latest pip version is installed. Otherwise, the next installation step would not work.

    .. code-block:: bash

            $ python -m pip install --upgrade pip

#. Compile and Install the CommonRoad Drivability Checker library by running

        .. code-block:: bash
            
            $ BUILD_JOBS=8 python setup.py build
            $ pip install .

   **Note:** This will automatically build all C++ dependencies of the CommonRoad
   Drivability Checker. The number 8 in this example indicates the number of CPU cores to be used for the compilation. Each job (possibly) will use a core, so specify this number according to your system and free cores. Advanced build settings can be modified in the file setup_options.py.

  Canceling the build and then restarting it should generally be safe,
  however make sure that the Python environment you activated stays the same.
  In case of any errors, try deleting the ```build`` folder and running pip again.
