.. _installation_cpp:

Installation of the C++ interface
---------------------------------

**Note:**
The following steps are necessary if you intend to use the CommonRoad Drivability
Checker from a C++ project. They are *not* necessary if you only intend to use
the CommonRoad Drivability Checker from Python.

We recommend consuming the CommonRoad drivability checker via the CMake `FetchContent` module.
Alternatively, you can also install the CommonRoad Drivability Checker as a library.
In any case, we first need to install essential and optional third party libraries,

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

Installation of Optional Third Party Libraries
**********************************************

For the installation of CGAL, please refer to `their website <https://github.com/CGAL/cgal>`_.

Using the CommonRoad Drivability Checker via CMake FetchContent (recommended)
*****************************************************************************

Recent CMake versions provide the `FetchContent` module which greatly simplifies the integration in many scenarios.
Simply insert the following snippet somewhere in your `CMakeLists.txt`:

.. code-block:: cmake

    include(FetchContent)

    FetchContent_Declare(
            CommonRoadDC
            SYSTEM  # Treat this as a system package, i.e., don't issue warnings from its headers
            GIT_REPOSITORY https://github.com/CommonRoad/commonroad-drivability-checker.git
            # You can specify any reference here, but prefer specifying a concrete commit if possible
            # as that will speed up the build since Git won't need to check whether the branch moved in the meantime
            GIT_TAG <reference to commit, branch, tag...>
    )
    FetchContent_MakeAvailable(CommonRoadDC)

Then, add the CommonRoad Drivability Checker as a dependency to the targets which require it:

.. code-block:: cmake

    target_link_libraries(<MyLibraryOrExecutable> PUBLIC CommonRoadDC::crcc)


Installation of the CommonRoad Drivability Checker as a library (not recommended)
*********************************************************************************

After installing all essential third party libraries, you can now install the CommonRoad Drivability Checker.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

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

            $ cmake -DCMAKE_INSTALL_PREFIX=/install/prefix/see/note/above -DCMAKE_BUILD_TYPE=Release -B build-cmake -S .
            $ cmake --build build-cmake -j JOB_COUNT

        **Note that you have to replace**
         - *JOB_COUNT*  with the number of jobs you are willing to allocate to cmake, for example *-j 2*.
           Each job (possibly) will use a core, so specify this number according to your system and free cores.

        We recommend using the ``Ninja`` build system for building the CommonRoad Drivability Checker as it is
        generally faster than the default Makefile build system.
        Moreover, Ninja supports parallel builds out of the box, so you don't need to specify the ``-j JOB_COUNT`` option.
        To use Ninja, you can install it via your package manager (e.g., ``apt-get install ninja-build`` on Debian-based systems)
        and then pass ``-G Ninja`` to the first CMake invocation above.


#. Install the CommonRoad Drivability Checker library by running

        .. code-block:: bash

            $ cmake --install build-cmake

