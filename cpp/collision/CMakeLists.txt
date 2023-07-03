### Definition of crcc

find_package(Boost REQUIRED)


set(COLLISION_SRC_ROOT ${COLLISION_ROOT}/src)

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

${COLLISION_SRC_ROOT}/tests/online_tests/broadphase_test.cc
${COLLISION_SRC_ROOT}/tests/online_tests/test_common.cc
)


add_library(crcc STATIC ${COLLISION_SRC})

#target_compile_definitions(crcc PRIVATE -DENABLE_COLLISION_TESTS=1)

target_compile_features(crcc
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET crcc PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(crcc
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${COLLISION_ROOT}/include>
    PRIVATE
        ${COLLISION_ROOT}/include
        ${Boost_INCLUDE_DIRS} 
    )

if(NOT TARGET s11n::s11n)
    message(STATUS "Serialization and pickling support have been disabled for crcc")
    target_compile_definitions(crcc PRIVATE -DENABLE_SERIALIZER=0)
else()
    message(STATUS "Serialization and pickling support have been enabled for crcc")
    target_link_libraries(crcc PUBLIC s11n::s11n)
    target_compile_definitions(crcc PRIVATE -DENABLE_SERIALIZER=1)
endif()

if(ADD_TRIANGLE)
    target_compile_definitions(crcc PUBLIC ENABLE_TRIANGLE=1)
endif()

target_link_libraries(crcc
  PRIVATE
    gpc
  PUBLIC
    fcl
    ccd
    Eigen3::Eigen
    box2d
)

if(ADD_TRIANGLE)
    target_link_libraries(crcc PRIVATE triangle)
endif()
