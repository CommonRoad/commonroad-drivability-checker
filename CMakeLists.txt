
project(DrivabilityChecker)
cmake_minimum_required(VERSION 3.18)

option(BUILD_SHARED_LIBS "Build dc as a shared library" OFF)

# commonroad_dc flags
option(ADD_TESTS "unit tests" OFF)
option(BUILD_DOC "generate the doc target." OFF)
option(ADD_MODULE_GEOMETRY "add geometry module" ON)
option(ADD_MODULE_COLLISION "add collision module" ON)
option(ADD_PYTHON_BINDINGS "add Python bindings" OFF)
option(ADD_TRIANGLE "Use a non-free Triangle library" OFF)

# FIXME Maybe rename this option in the future?
# Note: BUILD_S11N is already used by other projects
option(BUILD_S11N "enable serialization support using s11n" ON)

option(BUILD_PYBIND11 "build pybind11 for python bindings" ON)

if("${CMAKE_BINARY_DIR}" EQUAL "${PROJECT_SOURCE_DIR}/build")
  message(WARNING
    "The 'build' directory inside the project root directory is also used by the Python build system!\n"
    "While the file and directory names used by CMake and setuptools/Python normally don't clash "
    "you should still consider using a different build directory.\n"
    "CMAKE_BINARY_DIR is set to ${CMAKE_BINARY_DIR}"
    )
endif()

# TODO: Investigate migrating setup.py to skbuild,
# a build system specifically for C/C++ Python extension modules

# Users should generally not use ADD_PYTHON_BINDINGS directly
mark_as_advanced(ADD_PYTHON_BINDINGS)

if(ADD_PYTHON_BINDINGS)
  message(STATUS "PYTHON MODE - assuming we are invoked by pip/setup.py")
  message(STATUS "PYTHON MODE - building static libraries")
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

include(FetchContent)
FetchContent_Declare(
    commonroad_cmake

    GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

# Disable verbose git clone output if we are invoked by Python
# The Git progress messages don't work correctly when relayed by Python
#
# Explanation: Verbose Git output is enabled iff GIT_PROGRESS=ON *and* FETCHCONTENT_QUIET=OFF
# We set GIT_PROGRESS to ON in every FetchContent_Declare call, and set FETCHCONTENT_QUIET to off
# if we are invoked by Python.
#
# References:
# https://cmake.org/cmake/help/v3.24/module/FetchContent.html#variable:FETCHCONTENT_QUIET
# https://cmake.org/cmake/help/v3.24/module/ExternalProject.html?highlight=git_progress
set(FETCHCONTENT_QUIET ${ADD_PYTHON_BINDINGS})

include(third_party/external_libccd_fcl.cmake)

if(BUILD_PYBIND11)
    include(third_party/external_pybind11.cmake)
endif()

include(third_party/external_box2d.cmake)

include(third_party/external_triangle.cmake)

include(third_party/external_gpc.cmake)

if(BUILD_S11N)
    add_subdirectory(third_party/libs11n)

    # FIXME Curently we use s11n_FOUND in various places to check whether we are using s11n
    # since we previously used find_package/ExternalProject.
    # Uses of s11n_FOUND should simply be changed to BUILD_S11N
    set(s11n_FOUND TRUE)
endif()

add_subdirectory(cpp)
