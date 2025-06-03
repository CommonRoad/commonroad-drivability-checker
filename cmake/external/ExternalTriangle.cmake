include_guard(DIRECTORY)

include(FetchContent)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(
  triangle

  SYSTEM

  URL            https://github.com/drufat/triangle/archive/3eda880cda23e0146570223816e278c7074bfcc5.zip
  URL_HASH       SHA256=c2e10f61a6989b136c44f905f4a15f1b0f2e7fa38147d46d3b6e6284fd9fd05f

  # CMake definitions for the triangle library are contained in the c subdirectory
  SOURCE_SUBDIR  c
  )

FetchContent_MakeAvailable(triangle)

# Static library for triangle sources

message(STATUS "Building with support for triangulation using a non-free Triangle library")

# NOTE: triangle's CMake script already sets TRILIBRARY and ANSI_DECLARATORS

# FIXME: These definitions shouldn't be required, perhaps they can be removed?
target_compile_definitions(triangle PRIVATE -DREAL=double -DVOID=int)

set_property(TARGET triangle PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(triangle PUBLIC $<BUILD_INTERFACE:${triangle_SOURCE_DIR}/c>)

