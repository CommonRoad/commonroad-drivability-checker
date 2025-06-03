include_guard(DIRECTORY)

include(FetchContent)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(
  box2d

  SYSTEM

  URL            https://github.com/erincatto/box2d/archive/refs/tags/v2.4.1.tar.gz
  URL_HASH       SHA256=d6b4650ff897ee1ead27cf77a5933ea197cbeef6705638dd181adc2e816b23c2
  )

set(BOX2D_BUILD_DOCS OFF CACHE BOOL "")
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "")
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "")

FetchContent_MakeAvailable(box2d)

