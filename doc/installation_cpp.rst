.. _installation_cpp:

Installation of the C++ interface
---------------------------------

**Note:**
The following steps are necessary if you intend to use the CommonRoad Drivability
Checker from a C++ project. They are *not* necessary if you only intend to use
the CommonRoad Drivability Checker from Python.

In order to install CommonRoad Drivability Checker, we first need to install essential third party libraries,
followed by installation of optional third party libraries, and finally the installation of CommonRoad Drivability
Checker itself. All required steps are described in the following sections.

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

*N.B.* If you are using Conda, make sure your environment is activated!

Installation of Optional Third Party Libraries
**********************************************

For the installation of CGAL, please refer to `their website <https://github.com/CGAL/cgal>`_.

Installation of the CommonRoad Drivability Checker
**************************************************

After installing all essential third party libraries and packages, you can now install the CommonRoad Drivability Checker.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Create a build directory and change into it

    .. code-block:: bash

            $ mkdir build-debug
            $ cd build-debug

   **NB:** You can in theory use ``build`` as a name for the build folder,
   however keep in mind that Python's setuptools will always use the ``build`` folder
   for their build process.
   This normally shouldn't cause any issues since the file names
   used by setuptools don't clash with any names CMake uses currently,
   but you should still consider using separate folders just in case.

#. Decide on a *install prefix path* for the following commands.
   The Drivability Checker uses a superbuild system for first building
   the bundled third-party dependencies and then building the Drivability
   Checker itself.

   By default, CMake chooses ``/usr/local`` as
   the install prefix on Linux, a directory which normal users can't write
   to (necessitating ``sudo``). This is not a good idea however if your intention
   is to build the Drivability Checker for local development.

   Instead, consider using a sibling directory to the root Drivability Checker
   directory as the install prefix: For example, if you cloned the
   Drivability Checker repository to the path ``~/commonroad/commonroad-drivability-checker``,
   the suggested install prefix would be ``~/commonroad/dist``.

   If you want to use the CommonRoad Drivability Checker as a dependency in
   another project, you can then use ``find_package(DrivabilityChecker CONFIG)``
   to discover the installed CommonRoad Drivability Checker.
   By setting ``CMAKE_PREFIX_PATH`` to the install prefix you gave to the
   CommonRoad Drivability Checker when invoking CMake on the downstream project,
   CMake will automatically discover it.

#. Compile the CommonRoad Drivability Checker library by running

        .. code-block:: bash

            $ cmake -DCMAKE_INSTALL_PREFIX=/install/prefix/see/note/above -DCMAKE_BUILD_TYPE=Release ..
            $ cmake --build . -- -j JOB_COUNT
        
        **Note that you have to replace**
         - *JOB_COUNT*  with the number of jobs you are willing to allocate to cmake, for example *-j 2*.
           Each job (possibly) will use a core, so specify this number according to your system and free cores. 


#. Install the CommonRoad Drivability Checker library by running

        .. code-block:: bash

            $ cmake --install .

