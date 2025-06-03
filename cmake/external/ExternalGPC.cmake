include_guard(DIRECTORY)

include(FetchContent)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(
  gpc

  SYSTEM

  URL https://github.com/rickbrew/GeneralPolygonClipper/archive/2dfa0579e988de86c245cc86ccb9092712665653.zip
  URL_HASH SHA256=73461b0cd9e964d6634eb2fc7269cbff377d4728d210eaed7573e95b0450a5b3
  )

FetchContent_MakeAvailable(gpc)

# Static library for gpc sources

add_library(gpc STATIC
  ${gpc_SOURCE_DIR}/gpc.c
  ${gpc_SOURCE_DIR}/gpc.h
  )

# fscanf_s is Windows-specific
target_compile_definitions(gpc PRIVATE $<$<NOT:$<PLATFORM_ID:Windows>>:fscanf_s=fscanf>)

set_property(TARGET gpc PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(gpc INTERFACE $<BUILD_INTERFACE:${gpc_SOURCE_DIR}>)
