# Notes for Developers

Here we provide more detailed information for using our software for development purposes.


## Table of Contents
- [Building from Source](#building-from-source)
- [Editable Installation](#editable-installation)
- [Debugging with an IDE](#debugging-with-an-ide)
- [Running Unit Tests](#running-unit-tests)


## Building from Source

### Build dependencies
**Manual installation:**
- [OpenMP](https://www.openmp.org/)

**Optional installation:** The following dependencies are automatically installed via CMake FetchContent.
However, to speed up the build we recommend to optionally install them manually.
- [Eigen3](https://eigen.tuxfamily.org/dox/)
- [Boost](https://www.boost.org/)
- [Box2D](https://github.com/erincatto/box2d)
- [FCL](https://github.com/flexible-collision-library/fcl)
- [libccd](https://github.com/danfis/libccd)
- [gpc](https://github.com/rickbrew/GeneralPolygonClipper)

The additional Python build dependencies are listed in [pyproject.toml](pyproject.toml) under `build`.


## Editable Installation
1. Install the aformenetioned C++ dependencies. 

2. Install the Python build dependencies (required to make `--no-build-isolation` work in the next step):
```bash
pip install "scikit-build-core~=0.11.0" "nanobind~=2.2.0" "pathspec>=0.12.1" "pyproject-metadata>=0.7.1" "typing_extensions~=4.12.2" "cmake (>=3.24, <4.0)"
```

> **Note:** The versions of the dependencies might have changed from the time of writing this README. Please check the
> optional build dependencies in the [`pyproject.toml`](../pyproject.toml) file for the latest versions.

3. Build the package and install it in editable mode with automatic rebuilds.
```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true -e .
```

Please check the [scikit-build-core documentation](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#editable-installs) for more details.

Flags:
- `-v` (verbose) output about the build progress
- `--no-build-isolation` disables build isolation, build runs in your local environment
- `--config-settings=editable.rebuild=true` enables automatic rebuilds when the source code changes
- `-e` editable install 


## Debugging the code
1. Install in editable mode using the CMake Debug build flag:
```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true --config-settings=cmake.build-type="Debug" -e .
```

2. Launch the Python interpreter together with a C++ debugger (e.g., GDB):
```bash
gdb -ex r --args python your_script.py
```

## Building Python Bindings Directly

Building the Python bindings directly, i.e., without using scikit-build-core, can be helpful e.g. to set up your IDE.
To do so, you need to add the following parameters to your CMake invocation.
```
-DCR_DC_BUILD_PYTHON_BINDINGS=ON
-DCMAKE_PREFIX_PATH=/path/to/site-packages
```
The first parameter enables the Python bindings for the Drivability Checker.
The second parameters adds the path to your Python installation's `site-packages` directory to the CMake search path like scikit-build-core does.
If you are using an Anaconda/Miniconda environment, make sure to point this to the `site-packages` directory of the correct environment.
Please make sure that `pybind11` with the version specified in the `build-system.requires` is installed in this environment.


## Building the Documentation

To build the documentation, make sure that the drivability checker Python package and its `docs` dependencies are installed as specified in `pyproject.toml`.
Moreover, you need to install `doxygen` and `pandoc` on your system.
Then, you can build the documentation by running:
```bash
cmake -S . -B build-docs -DCR_DC_BUILD_DOCS=ON
cmake --build build-docs --target Sphinx
```
