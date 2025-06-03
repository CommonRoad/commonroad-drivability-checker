set(COLLISION_SRC
        src/raytrace_primitive.cc
        src/shape_group.cc
        src/collision_object.cc
        src/collision_checker.cc
        src/primitive_collision_checker.cc
        src/raytrace_utils.cc
        src/collision_object_ex.cc
        src/time_variant_collision_object.cc
        src/solvers/distance_queries.cc
        src/solvers/detail/collision_containers.cc
        src/solvers/collision_queries.cc
        src/solvers/trajectory_queries.cc
        src/solvers/geometry_queries.cc
        src/solvers/collision_solver_default.cc
        src/solvers/collision_solver_fcl.cc
        src/solvers/collision_solver_primitive.cc

        src/solvers/fcl/fcl_collision_checker.cc
        src/solvers/fcl/fcl_distance_queries.cc
        src/solvers/fcl/fcl_broadphase_manager_factories.cc
        src/solvers/fcl/fcl_distance_requests.cc
        src/solvers/fcl/fcl_collision_queries.cc
        src/solvers/fcl/fcl_collision_object_group.cc
        src/solvers/fcl/fcl_collision_requests.cc
        src/solvers/fcl/fcl_collision_object.cc
        src/solvers/fcl/fcl_entity_factory.cc
        src/solvers/fcl/performance_timers.cc
        src/solvers/fcl/solver_entity_fcl.cc

        src/solvers/boost/solver_entity_boost.cc
        src/solvers/boost/boost_collision_object.cc
        src/solvers/boost/boost_object_polygon.cc
        src/solvers/boost/boost_collision_queries.cc
        src/solvers/boost/boost_geometry_queries.cc

        src/plugins/triangulation/triangulate.cc

        src/serialize/serialize_rectangle_aabb.cc
        src/serialize/serialize_collision_checker.cc
        src/serialize/serialize_rectangle_obb.cc
        src/serialize/serialize_tv_object.cc
        src/serialize/serialize_point.cc
        src/serialize/serialize_triangle.cc
        src/serialize/serialize_container_cc_obj.cc
        src/serialize/serialize_container_obj_obj.cc
        src/serialize/serialize_polygon.cc
        src/serialize/serialize_shape_group.cc
        src/serialize/serialize.cc
        src/serialize/serialize_sphere.cc
        src/serialize/public/serialize_public.cc
        src/serialize/final/collision_object_export_final.cc
        src/narrowphase/polygon.cc
        src/narrowphase/rectangle_aabb.cc
        src/narrowphase/triangle.cc
        src/narrowphase/rectangle_obb.cc
        src/narrowphase/shape.cc
        src/narrowphase/point.cc
        src/narrowphase/sphere.cc
        src/narrowphase/detail/aabb.cc
        src/solvers/sat2d/sat2d_checks.cc

        src/tests/online_tests/broadphase_test.cc
        src/tests/online_tests/test_common.cc
)

set(COLLISION_HDR
        include/collision/collision_object_types.h
        include/collision/i_collision_object_export.h
        include/collision/i_collision_checker_export.h
        include/collision/raytrace_primitive.h
        include/collision/collision_object.h
        include/collision/i_collision_container.h
        include/collision/i_collision_checker.h
        include/collision/raytrace_utils.h
        include/collision/application.h
        include/collision/application_settings.h
        include/collision/shape_group.h
        include/collision/collision_object_ex.h
        include/collision/collision_checker.h
        include/collision/plugins/triangulation/triangulate.h
        include/collision/primitive_collision_checker.h
        include/collision/time_variant_collision_object.h
        include/collision/line_segment.h

        include/collision/solvers/accelerators/declarations.h
        include/collision/solvers/accelerators/detail/container_grid.h
        include/collision/solvers/accelerators/detail/container_grid_common.h
        include/collision/solvers/accelerators/detail/container_box2d.h
        include/collision/solvers/accelerators/detail/container_fcl.h
        include/collision/solvers/accelerators/detail/container_fcl_inl.h
        include/collision/solvers/accelerators/detail/container_grid_inl.h
        include/collision/solvers/accelerators/detail/container_box2d_inl.h
        include/collision/solvers/accelerators/detail/common.h
        include/collision/solvers/accelerators/detail/common_impl.h
        include/collision/solvers/collision_queries.h
        include/collision/solvers/collision_requests.h
        include/collision/solvers/trajectory_queries.h
        include/collision/solvers/geometry_queries.h
        include/collision/solvers/fcl/fcl_distance_queries.h
        include/collision/solvers/fcl/fcl_collision_requests.h
        include/collision/solvers/fcl/fcl_helpers.h
        include/collision/solvers/fcl/fcl_broadphase_manager_factory.h
        include/collision/solvers/fcl/solver_entity_fcl.h
        include/collision/solvers/fcl/fcl_collision_checker.h
        include/collision/solvers/fcl/performance_timers.h
        include/collision/solvers/fcl/fcl_transform.h
        include/collision/solvers/fcl/fcl_collision_object_group.h
        include/collision/solvers/fcl/i_solver_entity_fcl.h
        include/collision/solvers/fcl/fcl_collision_queries.h
        include/collision/solvers/fcl/solver_entity_fcl_factory.h
        include/collision/solvers/fcl/fcl_collision_object.h
        include/collision/solvers/fcl/fcl_distance_requests.h
        include/collision/solvers/fcl/i_fcl_collision_object_group.h
        include/collision/solvers/fcl/i_fcl_collision_object.h
        include/collision/solvers/fcl/fcl_decl.h
        include/collision/solvers/fcl/fcl_entity_type.h
        include/collision/solvers/distance_requests.h
        include/collision/solvers/collision_solver_primitive.h
        include/collision/solvers/distance_queries.h
        include/collision/solvers/primitive_collision_queries.h
        include/collision/solvers/collision_solver_fcl.h
        include/collision/solvers/sat2d/triangle_sat2d.h
        include/collision/solvers/sat2d/sat2d_checks.h
        include/collision/solvers/sat2d/aabb_sat2d.h
        include/collision/solvers/sat2d/obb_sat2d.h
        include/collision/solvers/boost/solver_entity_boost.h
        include/collision/solvers/boost/boost_entity_type.h
        include/collision/solvers/boost/boost_collision_object.h
        include/collision/solvers/boost/boost_geometry_queries.h
        include/collision/solvers/boost/boost_helpers.h
        include/collision/solvers/boost/i_boost_collision_object.h
        include/collision/solvers/boost/boost_object_internal.h
        include/collision/solvers/boost/boost_collision_queries.h
        include/collision/solvers/boost/solver_entity_boost_factory.h
        include/collision/solvers/boost/boost_object_polygon.h
        include/collision/solvers/boost/i_solver_entity_boost.h
        include/collision/solvers/collision_solvers.h

        include/collision/serialize/triangle_export.h
        include/collision/serialize/vector2d_export.h
        include/collision/serialize/shape_export.h
        include/collision/serialize/i_collision_object_export.h
        include/collision/serialize/final/collision_object_export_final.h
        include/collision/serialize/point_export.h
        include/collision/serialize/rectangle_aabb_export.h
        include/collision/serialize/i_collision_checker_export.h
        include/collision/serialize/vector2d_export_streams.h
        include/collision/serialize/polygon_export.h
        include/collision/serialize/collision_checker_export.h
        include/collision/serialize/shape_group_export.h
        include/collision/serialize/basic_types.h
        include/collision/serialize/rectangle_obb_export.h
        include/collision/serialize/export_structs/rectangle_obb_export_struct.h
        include/collision/serialize/export_structs/sphere_export_struct.h
        include/collision/serialize/export_structs/point_export_struct.h
        include/collision/serialize/export_structs/tv_object_export_struct.h
        include/collision/serialize/export_structs/rectangle_aabb_export_struct.h
        include/collision/serialize/export_structs/polygon_export_struct.h
        include/collision/serialize/export_structs/triangle_export_struct.h
        include/collision/serialize/export_structs/vertex_export_struct.h
        include/collision/serialize/broadphase_failure_export.h
        include/collision/serialize/collision_object_export_s11.h
        include/collision/serialize/public/serialize_public.h
        include/collision/serialize/broadphase_failure_cc_obj_export.h
        include/collision/serialize/broadphase_failure_obj_obj_export.h
        include/collision/serialize/tv_object_export.h
        include/collision/serialize/serialize.h
        include/collision/serialize/serialize_reg_impl.h
        include/collision/serialize/sphere_export.h

        include/collision/tests/test_common.h
        include/collision/tests/broadphase_test.h
        include/collision/tests/broadphase_failure.h
        include/collision/tests/collision_tests.h

        include/collision/narrowphase/point.h
        include/collision/narrowphase/rectangle_obb.h
        include/collision/narrowphase/shape.h
        include/collision/narrowphase/utils.h
        include/collision/narrowphase/sphere.h
        include/collision/narrowphase/polygon.h
        include/collision/narrowphase/rectangle_aabb.h
        include/collision/narrowphase/detail/aabb.h
        include/collision/narrowphase/detail/obb.h
        include/collision/narrowphase/triangle.h
)

add_library(crcc ${COLLISION_SRC})

target_sources(crcc PRIVATE ${CR_CLCS_HDR_FILES})

if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(crcc
            INTERFACE
            FILE_SET crcc_headers
            TYPE HEADERS
            BASE_DIRS include
            FILES ${CR_CLCS_HDR_FILES})
endif ()

#target_compile_definitions(crcc PRIVATE -DENABLE_COLLISION_TESTS=1)

target_compile_features(crcc PUBLIC cxx_lambdas cxx_auto_type)

target_include_directories(crcc
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
)

target_link_libraries(crcc
        PRIVATE
        gpc
)

target_link_libraries(crcc
        PUBLIC
        fcl
        ccd
        box2d
        Eigen3::Eigen
        Boost::align
        Boost::geometry
        Boost::headers
        Boost::polygon
)

if (APPLE)
    # Required to find library -lomp on mac
    # MAC_LIBOMP_PATH defined in root CMakeLists.txt
    target_link_directories(env_model INTERFACE "${MAC_LIBOMP_PATH}/lib")
endif ()

# Serialization and pickling support
if (CR_DC_BUILD_S11N)
    message(STATUS "Serialization and pickling support have been ENABLED for crcc")
    target_link_libraries(crcc PUBLIC s11n::s11n)
    target_compile_definitions(crcc PRIVATE -DENABLE_SERIALIZER=1)
else ()
    message(STATUS "Serialization and pickling support have been DISABLED for crcc")
    target_compile_definitions(crcc PRIVATE -DENABLE_SERIALIZER=0)
endif ()

if (CR_DC_USE_TRIANGLE)
    target_compile_definitions(crcc PUBLIC ENABLE_TRIANGLE=1)
    target_link_libraries(crcc PRIVATE triangle)
endif ()

add_library(CommonRoadDC::crcc ALIAS crcc)

if (${PROJECT_NAME}_IS_TOP_LEVEL)
    foreach (target IN ITEMS crcc)
        # this adds many warnings to the build. They usually help to find some bugs
        # TODO: Check whether each warning flag is actually supported by the compiler before adding it
        if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wconversion
                    -pedantic -Wfatal-errors -Wno-unused-parameter -Wno-unused-function -Wno-unused-but-set-variable)
        endif ()
        if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
            target_compile_options(${target} PRIVATE /W3)
        endif ()
    endforeach ()
endif ()
