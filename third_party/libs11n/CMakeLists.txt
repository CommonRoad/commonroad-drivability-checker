project(s11n)
cmake_minimum_required(VERSION 3.0)

set(S11_SRC_ROOT "./src")

set(S11_SRC 	

${S11_SRC_ROOT}/argv_parser.cpp
${S11_SRC_ROOT}/compact.flex.cpp
${S11_SRC_ROOT}/compact_serializer.cpp
${S11_SRC_ROOT}/data_node_io.cpp
${S11_SRC_ROOT}/exception.cpp
${S11_SRC_ROOT}/funtxt.flex.cpp
${S11_SRC_ROOT}/funtxt_serializer.cpp
${S11_SRC_ROOT}/funxml.flex.cpp
${S11_SRC_ROOT}/funxml_serializer.cpp
${S11_SRC_ROOT}/parens.flex.cpp
${S11_SRC_ROOT}/parens_serializer.cpp
${S11_SRC_ROOT}/path_finder.cpp
${S11_SRC_ROOT}/plugin.cpp
${S11_SRC_ROOT}/s11n.cpp
${S11_SRC_ROOT}/s11nlite.cpp
${S11_SRC_ROOT}/s11n_node.cpp
${S11_SRC_ROOT}/simplexml.flex.cpp
${S11_SRC_ROOT}/simplexml_serializer.cpp
${S11_SRC_ROOT}/strtool.cpp
${S11_SRC_ROOT}/wesnoth.flex.cpp
${S11_SRC_ROOT}/wesnoth_serializer.cpp
)

add_library(s11n SHARED ${S11_SRC})

target_compile_features(s11n
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)
set_property(TARGET s11n PROPERTY POSITION_INDEPENDENT_CODE ON)

set(S11N_INCLUDES "include" "src")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")

target_include_directories(s11n PUBLIC ${S11N_INCLUDES})

include(GNUInstallDirs)
install(TARGETS s11n
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


