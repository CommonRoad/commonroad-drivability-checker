if(ADD_MODULE_COLLISION)

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

set(TRIANGLE_SRC ${COLLISION_SRC_ROOT}/plugins/triangulation/triangle_wrapper.c)


add_definitions(-DANSI_DECLARATORS=1 -DTRILIBRARY=1)

find_path(BOX2D_SRC_ROOT_1 NAMES "b2_broad_phase.cpp" PATHS "../third_party/box2d/src/collision/" REQUIRED)

set(BOX2D_SRC_ROOT ${BOX2D_SRC_ROOT_1}/..)

set(BOX2D_SRC
	${BOX2D_SRC_ROOT}/collision/b2_broad_phase.cpp
	${BOX2D_SRC_ROOT}/collision/b2_collision.cpp
	${BOX2D_SRC_ROOT}/collision/b2_dynamic_tree.cpp
	${BOX2D_SRC_ROOT}/common/b2_block_allocator.cpp
	${BOX2D_SRC_ROOT}/common/b2_math.cpp
	${BOX2D_SRC_ROOT}/common/b2_settings.cpp
	${BOX2D_SRC_ROOT}/common/b2_stack_allocator.cpp
	${BOX2D_SRC_ROOT}/common/b2_timer.cpp
	)


set(LIB_CRCC_SRC ${LIB_CRCC_SRC} ${COLLISION_SRC} ${TRIANGLE_SRC} ${BOX2D_SRC})

find_package(fcl REQUIRED)

find_package(ccd REQUIRED)


if(NOT fcl_FOUND)
    message(FATAL_ERROR "Could NOT find required library libfcl")
endif()

add_library(crcc ${LIB_CRCC_SRC})

target_compile_features(crcc
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET crcc PROPERTY POSITION_INDEPENDENT_CODE ON)

set(COLLISION_INCLUDES "./collision/include" ".")
set(BOX2D_INCLUDES "../third_party/box2d/include")

set(TRIANGLE_INCLUDES "../third_party/triangle/c/")

set(S11N_INCLUDES "../third_party/libs11n/include")

target_include_directories(crcc PUBLIC ${COLLISION_INCLUDES} ${TRIANGLE_INCLUDES} ${BOX2D_INCLUDES} ${S11N_INCLUDES})

message(STATUS "Using serialization library from: ${S11N_LIBRARY}")

if(NOT S11N_LIBRARY)
	message(STATUS "Serialization and pickling support have been disabled for the C++ library")
	set(S11N_LIBRARY "")
	add_definitions(-DENABLE_SERIALIZER=0)
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
endif()

target_link_libraries(crcc PUBLIC ccd fcl ${S11N_LIBRARY})

endif()

if(ADD_MODULE_GEOMETRY)

set(LIB_CRCCOSY_SRC
        geometry/segment.cc
        geometry/curvilinear_coordinate_system.cc
        geometry/util.cc
        
        )

set(LIB_CRCCOSY_INCLUDE ".")

add_library(crccosy ${LIB_CRCCOSY_SRC})

target_compile_features(crccosy
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET crccosy PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(crccosy PUBLIC ${LIB_CRCCOSY_INCLUDE})

endif()
