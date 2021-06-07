# CONF
project(DrivabilityChecker)
cmake_minimum_required(VERSION 3.12)

option(BUILD_SHARED_LIBS "Build s11n as a shared library" OFF)

option(ADD_TESTS "unit tests" OFF)
option(BUILD_DOC "generate the doc target." OFF)
option(ADD_MODULE_GEOMETRY "add geometry module" ON)
option(ADD_MODULE_COLLISION "add collision module" ON)
option(ADD_PYTHON_BINDINGS "add Python bindings" ON)

if(ADD_PYTHON_BINDINGS)
    find_package(Python COMPONENTS Interpreter Development REQUIRED)

    # Specific binary directory required here
    add_subdirectory(../third_party/pybind11 third_party/pybind11)
endif()

### Shared configuration

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Discover dependencies

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

if(ADD_MODULE_COLLISION AND BUILD_DOC)
  set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})
  add_subdirectory(doc)
endif()


find_package(OpenMP)

if(ADD_MODULE_COLLISION)
    if(ADD_TESTS)
        add_subdirectory(tests)
    endif()
endif(ADD_MODULE_COLLISION)

if(ADD_MODULE_COLLISION)

    find_package(fcl REQUIRED)

    find_package(ccd REQUIRED)

    find_package(box2d REQUIRED)

    find_package(s11n)

endif(ADD_MODULE_COLLISION)

### Definition of crcc

if(ADD_MODULE_COLLISION)

# Object library for triangle sources

add_library(triangle OBJECT
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c/triangle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c/triangle.h")

target_compile_definitions(triangle PRIVATE -DANSI_DECLARATORS=1 -DTRILIBRARY=1 -DREAL=double -DVOID=int)

set_property(TARGET triangle PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(triangle
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c
    )


set(COLLISION_SRC_ROOT "collision/src")

set(COLLISION_SRC
${COLLISION_SRC_ROOT}/raytrace_primitive.cc
${COLLISION_SRC_ROOT}/shape_group.cc
${COLLISION_SRC_ROOT}/collision_object.cc
${COLLISION_SRC_ROOT}/collision_checker.cc
${COLLISION_SRC_ROOT}/primitive_collision_checker.cc
${COLLISION_SRC_ROOT}/raytrace_utils.cc
${COLLISION_SRC_ROOT}/collision_object_ex.cc
${COLLISION_SRC_ROOT}/time_variant_collision_object.cc
${COLLISION_SRC_ROOT}/solvers/distance_queries.cc
${COLLISION_SRC_ROOT}/solvers/detail/collision_containers.cc
${COLLISION_SRC_ROOT}/solvers/collision_queries.cc
${COLLISION_SRC_ROOT}/solvers/trajectory_queries.cc
${COLLISION_SRC_ROOT}/solvers/geometry_queries.cc
${COLLISION_SRC_ROOT}/solvers/collision_solver_default.cc
${COLLISION_SRC_ROOT}/solvers/collision_solver_fcl.cc
${COLLISION_SRC_ROOT}/solvers/collision_solver_primitive.cc

${COLLISION_SRC_ROOT}/solvers/fcl/fcl_collision_checker.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_distance_queries.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_broadphase_manager_factories.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_distance_requests.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_collision_queries.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_collision_object_group.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_collision_requests.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_collision_object.cc
${COLLISION_SRC_ROOT}/solvers/fcl/fcl_entity_factory.cc
${COLLISION_SRC_ROOT}/solvers/fcl/performance_timers.cc
${COLLISION_SRC_ROOT}/solvers/fcl/solver_entity_fcl.cc

${COLLISION_SRC_ROOT}/solvers/boost/solver_entity_boost.cc
${COLLISION_SRC_ROOT}/solvers/boost/boost_collision_object.cc
${COLLISION_SRC_ROOT}/solvers/boost/boost_object_polygon.cc
${COLLISION_SRC_ROOT}/solvers/boost/boost_collision_queries.cc
${COLLISION_SRC_ROOT}/solvers/boost/boost_geometry_queries.cc

${COLLISION_SRC_ROOT}/plugins/triangulation/triangulate.cc

${COLLISION_SRC_ROOT}/serialize/serialize_rectangle_aabb.cc
${COLLISION_SRC_ROOT}/serialize/serialize_collision_checker.cc
${COLLISION_SRC_ROOT}/serialize/serialize_rectangle_obb.cc
${COLLISION_SRC_ROOT}/serialize/serialize_tv_object.cc
${COLLISION_SRC_ROOT}/serialize/serialize_point.cc
${COLLISION_SRC_ROOT}/serialize/serialize_triangle.cc
${COLLISION_SRC_ROOT}/serialize/serialize_container_cc_obj.cc
${COLLISION_SRC_ROOT}/serialize/serialize_container_obj_obj.cc
${COLLISION_SRC_ROOT}/serialize/serialize_polygon.cc
${COLLISION_SRC_ROOT}/serialize/serialize_shape_group.cc
${COLLISION_SRC_ROOT}/serialize/serialize.cc
${COLLISION_SRC_ROOT}/serialize/serialize_sphere.cc
${COLLISION_SRC_ROOT}/serialize/public/serialize_public.cc
${COLLISION_SRC_ROOT}/serialize/final/collision_object_export_final.cc
${COLLISION_SRC_ROOT}/narrowphase/polygon.cc
${COLLISION_SRC_ROOT}/narrowphase/rectangle_aabb.cc
${COLLISION_SRC_ROOT}/narrowphase/triangle.cc
${COLLISION_SRC_ROOT}/narrowphase/rectangle_obb.cc
${COLLISION_SRC_ROOT}/narrowphase/shape.cc
${COLLISION_SRC_ROOT}/narrowphase/point.cc
${COLLISION_SRC_ROOT}/narrowphase/sphere.cc
${COLLISION_SRC_ROOT}/narrowphase/detail/aabb.cc
${COLLISION_SRC_ROOT}/solvers/sat2d/sat2d_checks.cc


tests/collision/online_tests/broadphase_test.cc
tests/collision/online_tests/test_common.cc
)




# $<TARGET_OBJECTS:triangle>
add_library(crcc_obj OBJECT ${COLLISION_SRC})

target_compile_features(crcc_obj
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET crcc_obj PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(crcc_obj
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/collision/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/collision/include
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

if(OpenMP_CXX_FOUND)
    target_link_libraries(crcc_obj PRIVATE OpenMP::OpenMP_CXX)
endif()

if(NOT s11n_FOUND)
    message(STATUS "Serialization and pickling support have been disabled for crcc")
    target_compile_definitions(crcc_obj PRIVATE -DENABLE_SERIALIZER=0)
else()
    message(STATUS "Serialization using s11n enabled for crcc")
    target_link_libraries(crcc_obj PRIVATE s11n::s11n)
    target_compile_options(crcc_obj PRIVATE -Wno-deprecated)
endif()

target_link_libraries(crcc_obj
  PRIVATE
    triangle
  PUBLIC
    ccd
    fcl
    Eigen3::Eigen
    box2d::box2d
    )

add_library(crcc SHARED $<TARGET_OBJECTS:crcc_obj>)

endif(ADD_MODULE_COLLISION)

### Definition of crccosy

if(ADD_MODULE_GEOMETRY)

set(LIB_CRCCOSY_SRC
        geometry/segment.cc
        geometry/curvilinear_coordinate_system.cc
        geometry/util.cc
        geometry/serialize/serialize_curvil_cs.cc
        geometry/serialize/public/serialize_public.cc
        )

add_library(crccosy_obj OBJECT ${LIB_CRCCOSY_SRC})

set_property(TARGET crccosy_obj PROPERTY POSITION_INDEPENDENT_CODE ON)

if(OpenMP_CXX_FOUND)
    target_link_libraries(crccosy_obj PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_libraries(crccosy_obj
    PUBLIC
        Eigen3::Eigen)

target_compile_features(crccosy_obj
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)

target_include_directories(crccosy_obj
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

if(NOT s11n_FOUND)
    message(STATUS "Serialization and pickling support have been disabled for crccosy")
    target_compile_definitions(crccosy_obj PRIVATE -DENABLE_SERIALIZER=0)
else()
    message(STATUS "Serialization using s11n enabled for crccosy")
    target_link_libraries(crccosy_obj PRIVATE s11n::s11n)
    target_compile_options(crccosy_obj PRIVATE -Wno-deprecated)
endif()

add_library(crccosy SHARED $<TARGET_OBJECTS:crccosy_obj>)

endif(ADD_MODULE_GEOMETRY)

if(ADD_PYTHON_BINDINGS)
    add_subdirectory(python_binding)
endif()

# Provides configure_package_config_file
include(CMakePackageConfigHelpers)

# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)


set(INSTALL_MODULES "")
if(ADD_MODULE_GEOMETRY)
    set(INSTALL_MODULES ${INSTALL_MODULES} crccosy pycrccosy)
endif()
if(ADD_MODULE_COLLISION)
    set(INSTALL_MODULES ${INSTALL_MODULES} crcc pycrcc)
endif()

install(TARGETS ${INSTALL_MODULES}
    EXPORT ${PROJECT_NAME}_Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake

  PATH_VARS CMAKE_INSTALL_DATAROOTDIR
  )

# Export target configuration (for installation)
install(EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  )

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  )

# Export target configuration (for local building)
export(EXPORT ${PROJECT_NAME}_Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  )

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/collision/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/geometry
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
  )