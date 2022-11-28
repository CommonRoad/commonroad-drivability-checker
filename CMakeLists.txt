project(DrivabilityChecker)
cmake_minimum_required(VERSION 3.10)

# Flags for enabling superbuild
option(BUILD_LIBCCD "build vendored libccd" ON)
option(BUILD_FCL "build vendored fcl" ON)
option(BUILD_S11N "build vendored s11n" ON)
option(BUILD_BOX2D "build vendored box2d" ON)

# commonroad_dc flags
option(ADD_TESTS "unit tests" OFF)
option(BUILD_DOC "generate the doc target." OFF)
option(ADD_MODULE_GEOMETRY "add geometry module" ON)
option(ADD_MODULE_COLLISION "add collision module" ON)
option(ADD_PYTHON_BINDINGS "add Python bindings" OFF)
option(ADD_TRIANGLE "Use a non-free Triangle library" OFF)


if("${CMAKE_BINARY_DIR}" EQUAL "${PROJECT_SOURCE_DIR}/build")
  message(WARNING
    "The 'build' directory inside the project root directory is also used by the Python build system!\n"
    "While the file and directory names used by CMake and setuptools/Python normally don't clash "
    "you should still consider using a different build directory.\n"
    "CMAKE_BINARY_DIR is set to ${CMAKE_BINARY_DIR}"
    )
endif()

include(ExternalProject)

set(SUPERBUILD_PREFIX_DIR  ${CMAKE_CURRENT_BINARY_DIR}/superbuild)
set(SUPERBUILD_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/install)
set_directory_properties(PROPERTIES EP_BASE ${SUPERBUILD_PREFIX_DIR})

set(DEPEND_LIBCCD "")
if(BUILD_LIBCCD)
  set(DEPEND_LIBCCD libccd)
  ExternalProject_Add(libccd
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/libccd"
      INSTALL_DIR       "${SUPERBUILD_INSTALL_DIR}"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DBUILD_DOCUMENTATION:BOOL=OFF
        -DBUILD_TESTING:BOOL=OFF
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
        -DENABLE_DOUBLE_PRECISION:BOOL=ON

  )
endif()

set(DEPEND_FCL "")
if(BUILD_FCL)
  set(DEPEND_FCL fcl)
  ExternalProject_Add(fcl
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/fcl"
      INSTALL_DIR       "${SUPERBUILD_INSTALL_DIR}"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_PREFIX_PATH=${SUPERBUILD_PREFIX_DIR}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DBUILD_TESTING:BOOL=OFF
        -DFCL_STATIC_LIBRARY:BOOL=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON	
      DEPENDS ${DEPEND_LIBCCD}
  )
endif()

set(DEPEND_S11N "")
if(NOT WIN32)
	if(BUILD_S11N)
	  set(DEPEND_S11N s11n)
	  ExternalProject_Add(s11n
		  SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/libs11n"
		  INSTALL_DIR       "${SUPERBUILD_INSTALL_DIR}"
		  CMAKE_ARGS
			-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
			-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
			-DBUILD_SHARED_LIBS:BOOL=ON
			-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
	  )
	endif()
endif(NOT WIN32)

set(DEPEND_BOX2D "")
if(BUILD_BOX2D)
  set(DEPEND_BOX2D box2d)
  ExternalProject_Add(box2d
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/box2d"
      INSTALL_DIR       "${SUPERBUILD_INSTALL_DIR}"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DBOX2D_BUILD_TESTBED:BOOL=OFF
        -DBOX2D_BUILD_DOCS:BOOL=OFF
        -DBOX2D_BUILD_UNIT_TESTS:BOOL=OFF
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  )
endif()

ExternalProject_Add(commonroad_dc
    SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/cpp"
    INSTALL_DIR       "${SUPERBUILD_INSTALL_DIR}"
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_PREFIX_PATH=<INSTALL_DIR>
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DBUILD_DOC:BOOL=${BUILD_DOC}
      -DADD_TESTS:BOOL=${ADD_TESTS}
      -DADD_MODULE_COLLISION:BOOL=${ADD_MODULE_COLLISION}
      -DADD_MODULE_GEOMETRY:BOOL=${ADD_MODULE_GEOMETRY}
      -DADD_PYTHON_BINDINGS:BOOL=${ADD_PYTHON_BINDINGS}
      -DADD_TRIANGLE:BOOL=${ADD_TRIANGLE}
      -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
      -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
      -DPYTHON_EXECUTABLE:PATH=${PYTHON_EXECUTABLE}
      -DPATH_TO_PYTHON_ENVIRONMENT:PATH=${PATH_TO_PYTHON_ENVIRONMENT}
      -DPYTHON_VER=${PYTHON_VER}
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      
    DEPENDS ${DEPEND_BOX2D} ${DEPEND_S11N} ${DEPEND_FCL}
    BUILD_ALWAYS TRUE
)

# Note: commonroad_dc and its dependencies will be installed to the SUPERBUILD_INSTALL_DIR
# during the build. This directory is only intended for internal usage.
#
# After the internal build completes, the whole internal install tree is installed
# using the usual CMake install command below (honoring CMAKE_INSTALL_PREFIX etc.).

install(
  DIRECTORY ${SUPERBUILD_INSTALL_DIR}/
  DESTINATION "."
  )
