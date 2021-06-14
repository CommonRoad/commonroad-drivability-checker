### Definition of crccosy

set(LIB_CRCCOSY_SRC
    src/segment.cc
    src/curvilinear_coordinate_system.cc
    src/util.cc
    serialize/serialize_curvil_cs.cc
    serialize/public/serialize_public.cc
  )

add_library(crccosy SHARED ${LIB_CRCCOSY_SRC})

set_property(TARGET crccosy PROPERTY POSITION_INDEPENDENT_CODE ON)

if(OpenMP_CXX_FOUND)
    target_link_libraries(crccosy PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_libraries(crccosy
    PUBLIC
        Eigen3::Eigen)

target_compile_features(crccosy
    PUBLIC
        cxx_lambdas cxx_auto_type
    )

target_include_directories(crccosy
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

if(NOT s11n_FOUND)
    message(STATUS "Serialization and pickling support have been disabled for crccosy")
    target_compile_definitions(crccosy PRIVATE -DENABLE_SERIALIZER=0)
else()
    message(STATUS "Serialization using s11n enabled for crccosy")
    target_link_libraries(crccosy PRIVATE s11n::s11n)
    target_compile_definitions(crccosy PRIVATE -DENABLE_SERIALIZER=1)
endif()