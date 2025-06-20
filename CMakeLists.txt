cmake_minimum_required(VERSION 3.20..3.31)

# Set Version (if SKBUILD: automatic retrieval of version)
if (SKBUILD)
    set(CR_DC_VERSION ${SKBUILD_PROJECT_VERSION})
else ()
    set(CR_DC_VERSION 2025.3.1)
endif ()

# Define project
project(CommonRoadDC
        LANGUAGES C CXX
        VERSION ${CR_DC_VERSION}
        HOMEPAGE_URL "https://commonroad.in.tum.de/tools/drivability-checker"
        DESCRIPTION "C++ extension for the CommonRoad Drivability Checker Python package")

# CMP0048 (3.0) - project() command sets version variables
# Relevant for fcl and ccd
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)

# CMP0077 (3.13) - option() honors normal variables.
# Relevant for Eigen3
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# CMP0126 (3.21) - Removal of normal variables by set(CACHE)
if (POLICY CMP0126)
    cmake_policy(SET CMP0126 NEW)
endif ()

# CMP0135 - URL download timestamp
if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif ()

# Adapted from Eigen3 - snippet to get a value for PROJECT_IS_TOP_LEVEL
# on CMake versions before v3.21.0
if (CMAKE_VERSION VERSION_LESS 3.21.0)
    if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        set(PROJECT_IS_TOP_LEVEL ON)
    else ()
        set(PROJECT_IS_TOP_LEVEL OFF)
    endif ()
    set(${PROJECT_NAME}_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif ()

if (NOT SKBUILD)
    set(CMAKE_VERIFY_INTERFACE_HEADER_SETS ON)
endif ()

# Ugly hack to make CMake discover libomp from Homebrew on Github Actions
if (APPLE)
    if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
        set(MAC_LIBOMP_PATH /opt/homebrew/opt/libomp)
    else ()
        set(MAC_LIBOMP_PATH /usr/local/opt/libomp)
    endif ()

    if (NOT EXISTS ${MAC_LIBOMP_PATH})
        message(FATAL_ERROR "could not find OpenMP path!")
    endif ()

    list(APPEND CMAKE_PREFIX_PATH ${MAC_LIBOMP_PATH})

    message(STATUS "OpenMP prefix: ${MAC_LIBOMP_PATH}")

    find_path(_omp_include_dir
            NAMES omp.h
            PATHS ${_omp_path}/include
            REQUIRED
    )
    message(STATUS "OpenMP include dir: ${_omp_include_dir}")

    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_${_lang}_COMPILER_ID MATCHES "AppleClang")
        foreach (_lang IN ITEMS C CXX)
            set(OpenMP_${_lang}_LIB_NAMES "omp")
            set(OpenMP_${_lang}_FLAGS "-Xclang -fopenmp")
            set(OpenMP_${_lang}_INCLUDE_DIR ${MAC_LIBOMP_PATH}/include)
        endforeach ()
    endif ()

    find_library(OpenMP_omp_LIBRARY
            NAMES omp
            PATHS ${MAC_LIBOMP_PATH}/lib
    )

    set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON)
endif ()

# Disable PCH on platforms other than Clang on Linux (spotty support)
if (NOT LINUX OR NOT (CMAKE_CXX_COMPILER_ID STREQUAL "Clang") OR DEFINED ENV{CIBUILDWHEEL})
    set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON)
endif ()

if (APPLE)
    # Disable shared libraries on apple --> Causes linker error with omp
    message(STATUS "Setting BUILD_SHARED_LIBS=OFF on MacOS")
    set(BUILD_SHARED_LIBS OFF)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE=1")
endif ()

set(CMAKE_COLOR_DIAGNOSTICS ON)

# Compile command database is required for Clang-assisted parsing in Doxygen
# TODO: Set this conditionally
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

if (CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif ()

option(CR_DC_BUILD_PYTHON_BINDINGS "Build Python bindings for Drivability Checker" OFF)

# IMPORTANT: DO NOT MOVE this section, in particular the calls to find_package(Python)
# and find_package(pybind11), without careful consideration
if (CR_DC_BUILD_PYTHON_BINDINGS)
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

    execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" -m nanobind --cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
    find_package(nanobind CONFIG REQUIRED)

    if (SKBUILD)
        # SKBUILD_SELF_CONTAINED controls whether we try to build all dependencies
        # ourselves. This is used in order to build cross-platform wheels
        # using cibuildwheel.
        # We don't enable this in normal Python builds since it will generally
        # just slow down the build.
        set(SKBUILD_SELF_CONTAINED OFF)

        if (DEFINED ENV{CIBUILDWHEEL})
            set(SKBUILD_SELF_CONTAINED ON)
        endif ()

        message(STATUS "PYTHON MODE - assuming we are invoked by pip/setup.py")
        message(STATUS "PYTHON MODE - building static libraries")

        set(FETCHCONTENT_QUIET ON)

        # Globally build static libraries (affects all calls to add_library
        # without an explicit library type)
        set(BUILD_SHARED_LIBS OFF)
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)

        # Globally set visibility preset
        set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
        set(CMAKE_CXX_VISIBILITY_PRESET default)
    else ()
        message(STATUS "PYTHON MODE - adding Python interface for compilation only")
    endif ()
endif (CR_DC_BUILD_PYTHON_BINDINGS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Some extra debugging for project developers - safe to disable
set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)
set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

# Ensure executables are in the top level directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(CMakeDependentOption)

# Build extras (docu, tests, python bindings)
set(CR_DC_BUILD_EXTRAS_DEFAULT ON)
if (NOT CommonRoadDC_IS_TOP_LEVEL)
    set(CR_DC_BUILD_EXTRAS_DEFAULT OFF)
endif ()

# Build tests
cmake_dependent_option(CR_DC_BUILD_TESTS
        "Build tests"
        ${CR_DC_BUILD_EXTRAS_DEFAULT}
        "NOT SKBUILD"
        OFF
)

# Build documentation
cmake_dependent_option(CR_DC_BUILD_DOCS
        "Build documentation"
        OFF
        "NOT SKBUILD"
        OFF
)
cmake_dependent_option(CR_DC_SPHINX_SEARCH_PATH
        "Additional directories to search for Sphinx executable"
        ""
        "CR_DC_BUILD_DOCS"
        ""
)

# Build S11n for serialization
option(CR_DC_BUILD_S11N
        "Enable serialization support using libs11n"
        ${CR_DC_BUILD_EXTRAS_DEFAULT}
)

# Use non-free Triangle library
option(CR_DC_USE_TRIANGLE
        "Use a non-free Triangle library"
        OFF
)

# Build as shared library
option(BUILD_SHARED_LIBS "Build commonroad DC as a shared library" ON)
option(CR_DC_BUILD_SHARED_LIBS "Build using shared libraries" ${BUILD_SHARED_LIBS})

# CMake find packages (depending on CMake version)
set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE OFF)
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else ()
    message(WARNING "Your CMake version (${CMAKE_VERSION}) does not support "
            "the FetchContent find_package integration introduced in CMake 3.24. "
            "As a fallback, we will simply build all dependencies ourselves "
            "irrespective of whether a suitable system version exists. "
            "While this does not impair functionality, it might slow down the build "
            "process a bit.\n"
            "In case you have all required dependencies installed, you can try "
            "enabling the option\n"
            "\tCR_DC_SYSTEM_PACKAGES_FORCE\n"
            "which will force using find_package for all dependencies.")
endif ()

# Option: Use system-wide installed packages for dependencies
option(CR_DC_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(CR_DC_SYSTEM_PACKAGES_FORCE
        "For CMake<3.24: Force using system packages for all dependencies"
        OFF
        "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
        OFF
)

# make FetchContent_Declare_Fallback available
include(FetchContent)
FetchContent_Declare(
        commonroad_cmake

        GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
        GIT_TAG main
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

include(toolchain/DiscoverLLD OPTIONAL)
include(toolchain/DiscoverSanitizers OPTIONAL)

# This is a helper script that will automatically add a .gitignore file to the
# binary directory (build directory) so you don't have to do add every build folder
# to your .gitignore.
include(extras/GitIgnoreBinaryDir OPTIONAL)

if (DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif ()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external)

# We have custom logic to check for system Boost, therefore we include ExternalBoost unconditionally
include(external/ExternalBoost)

include(ExternalBox2D)
include(ExternalGPC)

# Only add the non-free Triangle library if the user explicitly requested it
if (CR_DC_USE_TRIANGLE)
    include(ExternalTriangle)
endif ()

if (CMAKE_SUPPORTS_TRY_FIND_PACKAGE)
    if (SKBUILD_SELF_CONTAINED)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif ()
    if (CR_DC_SYSTEM_PACKAGES)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE OPT_IN)
    else ()
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif ()
endif ()

# Include Eigen3
if (CR_DC_SYSTEM_PACKAGES_FORCE)
    # This is the fallback branch in case the CMake version is older than 3.24
    # and the user requested we try to use system packages
    # For CMake > 3.24, fallback is automatic through the FetchContent find_package
    # integration.

    message(WARNING "CR_DC_SYSTEM_PACKAGES_FORCE is set - trying to satisfy "
            "all dependencies using installed system packages.\n"
            "If this fails, consider disabling CR_DC_SYSTEM_PACKAGES_FORCE and "
            "trying again.")

    find_package(Eigen3 3.3.7 REQUIRED)
else ()
    # Normal path: We try to use find_package via FetchContent, otherwise we fall
    # back to normal FetchContent

    include(external/ExternalEigen)
endif ()

# This needs to happen after we include the Eigen3 module, because it uses ${EIGEN3_INCLUDE_DIR}
include(ExternalLibccdFCL)

# Add third party subdirectory for building libs11n
if (CR_DC_BUILD_S11N AND NOT TARGET s11n::s11n)
    add_subdirectory(third_party/libs11n)
endif ()

# Add subdirectory for the main library
add_subdirectory(cpp)

# Add subdirectory for the Python bindings
if (CR_DC_BUILD_PYTHON_BINDINGS)
    add_subdirectory(python_binding)
endif ()

# Add subdirectory for tests
if (CR_DC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(cpp/tests)
endif ()

# Add subdirectory for documentation
if (CR_DC_BUILD_DOCS)
    add_subdirectory(doc)
endif ()

include(cmake/install.cmake)
