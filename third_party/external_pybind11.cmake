include_guard(DIRECTORY)

include(FetchContent)
include(utils/FetchContentHelper)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare_Fallback(
  pybind11

  URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz
  URL_HASH       SHA256=d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c

  FIND_PACKAGE_ARGS
  )

# set(PYBIND11_CPP_STANDARD -std=c++17)


FetchContent_MakeAvailable(pybind11)


