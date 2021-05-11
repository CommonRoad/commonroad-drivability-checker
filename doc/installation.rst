.. _installation:

Installation
-----------------

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

**Note: The installation script only works on Ubuntu based distributions. If your operating system is not Ubuntu
based, please refer to Method 2, which is described below.**

We have provided a build script for easy installation. You still need to activate your conda environment
(or virtual environment if you like). Full installation option will install third party libraries (suggested).

The **-i** option on the build command will install the drivability checker to the environment specified in the path.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your conda environment with the following command if you didn't before

    .. code-block:: bash

            $ conda activate commonroad-py36

#. Run the build script

        **Basic installation without CGAL and s11n:**

        .. code-block:: bash

            $ bash build.sh -e /path/to/your/anaconda3/envs/commonroad-py36 -v 3.6 -i -j JOB_COUNT

        **(SUGGESTED) Full installation:**

        .. code-block:: bash

            $ bash build.sh -e /path/to/your/anaconda3/envs/commonroad-py36 -v 3.6 --cgal --serializer -i -j JOB_COUNT

        **Note that you have to replace**
         - */path/to/your/anaconda3/envs/commonroad-py36* with the path to your Anaconda environment (or virtualenv);
         - *3.6*  with the Python version of your Anaconda environment.
         - *JOB_COUNT*  with the number of jobs you are willing to allocate to cmake, for example *-j 2*. Each job (possibly) will use a core, so specify this number according to your system and free cores.

        For additional options, please run **bash build.sh -h** command to view them.


Method 2: Manual Installation
#############################

Prerequisites
*************

We assume that the following libraries are already installed on your system:

* `Eigen3 <https://eigen.tuxfamily.org/dox/>`_ (version >=3.3.7 - preferably the newest version)
* `Boost libraries <https://www.boost.org/>`_ (version >= 1.58)
* `OpenMP <https://www.openmp.org/>`_ (for the geometry submodule support)

For the documentation, we require the libraries `Pandoc <https://pandoc.org>`__ and `Doxygen <http://www.doxygen.nl>`_.
All aforementioned libraries can be installed on Ubunutu via apt-get and on macOS via brew install (see homebrew).

Installation of Essential Third Party Libraries and Packages
************************************************************

Following third party libraries are included as submodules:

* `Box2D <https://github.com/erincatto/box2d>`_
* `FCL -- The Flexible Collision Library <https://github.com/flexible-collision-library/fcl>`_
* `libccd <https://github.com/danfis/libccd>`_
* `pybind11 <https://github.com/pybind/pybind11>`_
* `Triangle <https://pypi.org/project/triangle/>`_ (for the C++ library)

To download the libraries, run the following commands in the root folder of the CommonRoad Drivability Checker:

    .. code-block:: bash
    
            $ git submodule init
            $ git submodule update

#. Install `libccd <https://github.com/danfis/libccd>`_:

    .. code-block:: bash

            $ cd third_party/libccd
            $ mkdir build && cd build
            $ cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON .. 
            $ make
            $ sudo make install
 
#. Install `FCL -- The Flexible Collision Library <https://github.com/flexible-collision-library/fcl>`_: 
  
    .. code-block:: bash

            $ cd third_party/fcl

            linux: $ sudo apt-get install libboost-dev libboost-thread-dev libboost-test-dev libboost-filesystem-dev libeigen3-dev
            macOS: $ brew install eigen

    .. code-block:: bash

            $ mkdir build && cd build
            $ cmake ..
            $ make
            $ sudo make install


Following packages are available via `PyPi <https://pypi.org/>`_:

* `commonroad-io <https://pypi.org/project/commonroad-io/>`_
* `commonroad-vehicle-models <https://pypi.org/project/commonroad-vehicle-models/>`_
* `matplotlib <https://pypi.org/project/matplotlib/>`_
* `Shapely <https://pypi.org/project/Shapely/>`_
* `numpy <https://pypi.org/project/numpy/>`_
* `Jupyter <https://pypi.org/project/jupyter/>`_ 
* `Triangle <https://pypi.org/project/triangle/>`_ (Python bindings)
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

Installation of Optional Third Party Libraries
**********************************************

For the installation of CGAL, please refer to `their website <https://github.com/CGAL/cgal>`_.

To use the pickle feature of the collision checker, install s11n.net library by running:

    .. code-block:: bash

        $ cd third_party/libs11n
        $ mkdir build && cd build
        $ cmake .. -DCMAKE_BUILD_TYPE=Release
        $ make
        $ sudo make install

Installation of the CommonRoad Drivability Checker
**************************************************

After installing all essential third party libraries and packages, you can now install the CommonRoad Drivability Checker.

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your conda environment with

    .. code-block:: bash

            $ conda activate commonroad-py36
   
#. Compile the CommonRoad Drivability Checker library by running
    
        .. code-block:: bash
           
            $ mkdir build
            $ cd build
            $ cmake -DADD_PYTHON_BINDINGS=TRUE -DPATH_TO_PYTHON_ENVIRONMENT="/path/to/your/anaconda3/envs/commonroad-py36" -DPYTHON_VERSION="3.6" -DCMAKE_BUILD_TYPE=Release ..
            
        Note that with Python 3.8 you might have to use:

        .. code-block:: bash

            $ cmake -DADD_PYTHON_BINDINGS=TRUE -DPYTHON_EXECUTABLE=$(which python) -DCMAKE_BUILD_TYPE=Release ..

        The next line refers only to users of Mac OS X 10+:

        .. code-block:: bash

            $ sed -i '' 's!-lccd!/usr/local/lib/libccd.2.0.dylib!' python_binding/CMakeFiles/pycrcc.dir/link.txt

        .. code-block:: bash
            
            $ make

        **Note that you have to replace** 
         - *"/path/to/your/anaconda3/envs/commonroad-py36"* with the path to your Anaconda environment;
         - *"3.6"*  with the Python version of your Anaconda environment.

        
#. (Optional) Install the CommonRoad Drivability Checker with

    .. code-block:: bash
        
            $ cd ..
            $ python setup.py install
    
    **OR** add the root folder of the CommonRoad Drivability Checker to your Python-Interpreter. 
