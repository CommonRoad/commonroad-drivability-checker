include_guard(DIRECTORY)

include(FetchContent)

set(ccd_FOUND TRUE CACHE BOOL "hack to make FCL believe CCD is already installed" FORCE)


if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# We need a patch command for libccd, but CMake does not automatically provide one at the moment
# See https://cmake.org/cmake/help/v3.24/module/ExternalProject.html?highlight=patch_command
# and https://gitlab.kitware.com/cmake/cmake/-/issues/16854
#
# Instead we simply use git am, which applies a patch file as a commit
find_package(Git REQUIRED)


if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  set(_fetch_content_override_flag OVERRIDE_FIND_PACKAGE)
else()
  set(_fcl_extra_patch ${PROJECT_SOURCE_DIR}/third_party/patch/fcl-cmake-skip-findpackage-ccd.patch)

  unset(ccd_FOUND)
  set(ccd_FOUND TRUE CACHE BOOL "hack to make FCL believe CCD is already installed" FORCE)
endif()

FetchContent_Declare(
  ccd

  GIT_REPOSITORY https://github.com/danfis/libccd.git
  GIT_TAG        7931e764a19ef6b21b443376c699bbc9c6d4fba8

  # Necessary to run git am in CI environment (git am requires committer name and email)
  GIT_CONFIG user.name=cmake user.email=cmake@localhost

  PATCH_COMMAND ${GIT_EXECUTABLE}
      -C <SOURCE_DIR>
      am
      --ignore-date
      --
      ${PROJECT_SOURCE_DIR}/third_party/patch/libccd-cmake-fix-install-location.patch

  ${_fetch_content_override_flag}
  )

FetchContent_Declare(
  fcl
  
  GIT_REPOSITORY  https://github.com/flexible-collision-library/fcl.git
  GIT_TAG         43f9805445e73397077127556165f8af822c0383

  # Necessary to run git am in CI environment (git am requires committer name and email)
  GIT_CONFIG user.name=cmake user.email=cmake@localhost

  PATCH_COMMAND ${GIT_EXECUTABLE}
      -C <SOURCE_DIR>
      am
      --ignore-date
      --
      ${PROJECT_SOURCE_DIR}/third_party/patch/fcl-cmake-disable-warnings.patch
      # Used for CMake<3.24 (see above)
      ${_fcl_extra_patch}
  )

# BUILD_TESTING is used by libccd and FCL
set(BUILD_TESTING OFF CACHE BOOL "")

# set(BUILD_DOCUMENTATION OFF)
set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "")

# Explictily disable Octomap support - we don't use it,
# but FCL might find it sometimes, causing inconsistencies
set(FCL_WITH_OCTOMAP OFF CACHE BOOL "")
set(FCL_STATIC_LIBRARY ON CACHE BOOL "")

FetchContent_MakeAvailable(ccd fcl)

