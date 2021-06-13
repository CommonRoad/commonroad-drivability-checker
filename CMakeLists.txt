project(DrivabilityChecker)
cmake_minimum_required(VERSION 3.12)

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
option(ADD_PYTHON_BINDINGS "add Python bindings" ON)

include(ExternalProject)

set(DEPEND_LIBCCD "")
if(BUILD_LIBCCD)
  set(DEPEND_LIBCCD libccd)
  ExternalProject_Add(libccd
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/libccd"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
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
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DBUILD_TESTING:BOOL=OFF
        -DFCL_STATIC_LIBRARY:BOOL=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
      DEPENDS ${DEPEND_LIBCCD}
  )
endif()

set(DEPEND_S11N "")
if(BUILD_S11N)
  set(DEPEND_S11N s11n)
  ExternalProject_Add(s11n
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/libs11n"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  )
endif()

set(DEPEND_BOX2D "")
if(BUILD_BOX2D)
  set(DEPEND_BOX2D box2d)
  ExternalProject_Add(box2d
      SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/third_party/box2d"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
        -DBOX2D_BUILD_TESTBED:BOOL=OFF
        -DBOX2D_BUILD_DOCS:BOOL=OFF
        -DBOX2D_BUILD_UNIT_TESTS:BOOL=OFF
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  )
endif()

ExternalProject_Add(commonroad_dc
    SOURCE_DIR        "${CMAKE_CURRENT_LIST_DIR}/cpp"
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DBUILD_DOC:BOOL=${BUILD_DOC}
      -DADD_TESTS:BOOL=${ADD_TESTS}
      -DADD_MODULE_COLLISION:BOOL=${ADD_MODULE_COLLISION}
      -DADD_MODULE_GEOMETRY:BOOL=${ADD_MODULE_GEOMETRY}
      -DADD_PYTHON_BINDINGS:BOOL=${ADD_PYTHON_BINDINGS}
    DEPENDS ${DEPEND_BOX2D} ${DEPEND_S11N} ${DEPEND_FCL}
)
