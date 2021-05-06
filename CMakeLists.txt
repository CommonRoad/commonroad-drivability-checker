project(crcc)
cmake_minimum_required(VERSION 3.0)

option(ADD_TESTS "unit tests" OFF)
option(BUILD_DOC "generate the doc target." OFF)
option(ADD_MODULE_GEOMETRY "add geometry module" ON)
option(ADD_MODULE_COLLISION "add collision module" ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../commonroad_dc/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../commonroad_dc/)

find_package(Eigen3 3.0.5 QUIET CONFIG)

# If Eigen3Config.cmake is not found, use the FindEigen3.cmake module
if(NOT Eigen3_FOUND)
  find_package(Eigen3 3.0.5 QUIET MODULE)
  set(Eigen3_FOUND ON)
endif()

if(Eigen3_FOUND)
  set(COM_HAVE_EIGEN TRUE)
else()
  message(SEND_ERROR "EIGEN3 (>= 3.0.5) is required by the CollisionChecker")
  set(COM_HAVE_EIGEN FALSE)
endif()


if(ADD_MODULE_GEOMETRY)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-ignored-attributes)

if(APPLE)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -lomp -I/usr/local/opt/libomp/include -L/usr/local/opt/libomp/lib")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY /usr/local/opt/libomp/lib/libomp.dylib)
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -lomp -I/usr/local/opt/libomp/include -L/usr/local/opt/libomp/lib")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY /usr/local/opt/libomp/lib/libomp.dylib)
    endif()
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

include_directories(${EIGEN3_INCLUDE_DIR})

endif()

if(ADD_MODULE_COLLISION)
find_library(S11N_LIBRARY s11n HINTS ${PC_S11N_LIBRARY} )

if(ADD_TESTS)
  add_subdirectory(cpp/tests)
endif()

endif()

if(ADD_PYTHON_BINDINGS)
    if (${PYTHON_VERSION} GREATER_EQUAL 3.8)
        set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VERSION})
        set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VERSION}.so)
        set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VERSION})
    else()
        set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VERSION}m)
        set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VERSION}m.so)
        set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VERSION}m)
    endif()
    add_subdirectory(third_party/pybind11)
    add_subdirectory(python_binding)
endif()

add_subdirectory(cpp)

if(ADD_MODULE_COLLISION)
if (BUILD_DOC)
  set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})
  add_subdirectory(doc)
endif ()
endif()

