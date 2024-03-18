if(WIN32)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)
endif(WIN32)

# Add modules for discovering Sphinx etc.
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})

if(APPLE)
   # do nothing
else()

message(STATUS "Using full rpaths")
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:$ORIGIN/")
endif("${isSystemDir}" STREQUAL "-1")

endif()


if(ADD_PYTHON_BINDINGS)


	message(STATUS ${PYTHON_VER})

	if((PYTHON_VER) AND (PATH_TO_PYTHON_ENVIRONMENT))
		if (${PYTHON_VER} GREATER_EQUAL 3.8)
			set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VER})
			set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VER}.so)
			set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VER})
		else()
			set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VER}m)
			set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VER}m.so)
			set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VER}m)
		endif()
	else()
		if((NOT PYTHON_INCLUDE_DIR) OR (NOT PYTHON_LIBRARY) OR (NOT PYTHON_EXECUTABLE))
			#find_package(Python3 COMPONENTS Interpreter Development)
		endif()
	endif()



	message(STATUS ${PYTHON_INCLUDE_DIR})
	message(STATUS ${PYTHON_LIBRARY})
	message(STATUS ${PYTHON_EXECUTABLE})
endif()

### Shared configuration

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Discover dependencies

if(NOT WIN32)
#find_package(s11n)
endif(NOT WIN32)

find_package(Eigen3 REQUIRED)

#find_package(OpenMP)

OPTION (USE_OpenMP "Use OpenMP to enamble <omp.h>" ON)

# Find OpenMP
if(APPLE AND USE_OpenMP)

    if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
        set(MAC_LIBOMP_PATH /opt/homebrew/opt/libomp)
    else ()
        set(MAC_LIBOMP_PATH /usr/local/opt/libomp)
    endif ()

    list(APPEND CMAKE_PREFIX_PATH ${MAC_LIBOMP_PATH})

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        foreach(_lang IN ITEMS C CXX)
            set(OpenMP_${_lang}_LIB_NAMES "omp")
            set(OpenMP_${_lang}_FLAGS "-Xclang -fopenmp")
            set(OpenMP_${_lang}_INCLUDE_DIR ${MAC_LIBOMP_PATH}/include)
        endforeach()
    endif()

    find_library(OpenMP_omp_LIBRARY
        NAMES omp
        PATHS ${MAC_LIBOMP_PATH}/lib
    )

endif()

if(USE_OpenMP)
  find_package(OpenMP REQUIRED)
endif(USE_OpenMP)



#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif(OPENMP_FOUND)


# Targets/Python modules to be installed
set(INSTALL_MODULES "")
set(INSTALL_PYTHON_MODULES "")

# Provides configure_package_config_file
include(CMakePackageConfigHelpers)

# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

if(WIN32)

set(S11_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libs11n/windows/src)

set(S11_SRC

${S11_SRC_ROOT}/argv_parser.cpp
${S11_SRC_ROOT}/data_node_io.cpp
${S11_SRC_ROOT}/exception.cpp
${S11_SRC_ROOT}/path_finder.cpp
${S11_SRC_ROOT}/plugin.cpp
${S11_SRC_ROOT}/s11n.cpp
${S11_SRC_ROOT}/s11nlite.cpp
${S11_SRC_ROOT}/s11n_node.cpp
${S11_SRC_ROOT}/strtool.cpp
)

set(S11_PLUGINS

${S11_SRC_ROOT}/compact.flex.cpp
${S11_SRC_ROOT}/compact_serializer.cpp
)

set(s11n_FOUND ON)

add_library(s11n STATIC ${S11_SRC} ${S11_PLUGINS})

target_compile_features(s11n PUBLIC cxx_std_98)

set_property(TARGET s11n PROPERTY CXX_STANDARD 98)

set_property(TARGET s11n PROPERTY POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(s11n PRIVATE -DWIN32=1)

target_include_directories(s11n
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libs11n/windows/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libs11n/windows/include
    )

add_library(s11n::s11n ALIAS s11n)

endif(WIN32)

### Definition of crcc

if(ADD_MODULE_COLLISION)
    set(COLLISION_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/collision)
    include(${CMAKE_CURRENT_SOURCE_DIR}/collision/CMakeLists.txt)
    set(INSTALL_MODULES ${INSTALL_MODULES} crcc gpc box2d)
    if(ADD_TRIANGLE)
        set(INSTALL_MODULES ${INSTALL_MODULES} triangle)
    endif()
    set(INSTALL_PYTHON_MODULES ${INSTALL_PYTHON_MODULES} pycrcc)
endif(ADD_MODULE_COLLISION)

if(WIN32)
set(INSTALL_MODULES ${INSTALL_MODULES} s11n)
endif(WIN32)

### Definition of crccosy

if(ADD_MODULE_GEOMETRY)
    set(CRCCOSY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/geometry)
    include(${CMAKE_CURRENT_SOURCE_DIR}/geometry/CMakeLists.txt)


    set(INSTALL_MODULES ${INSTALL_MODULES} crccosy)
    set(INSTALL_PYTHON_MODULES ${INSTALL_PYTHON_MODULES} pycrccosy)
endif(ADD_MODULE_GEOMETRY)

if(BUILD_S11N)
    set(INSTALL_MODULES ${INSTALL_MODULES} s11n)
endif()

if(ADD_PYTHON_BINDINGS)
    set(PYBIND_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/python_binding)
    include(${CMAKE_CURRENT_SOURCE_DIR}/python_binding/CMakeLists.txt)

endif()

if(ADD_TESTS)
    add_subdirectory(tests)
endif()

if(ADD_PYTHON_BINDINGS)
    install(TARGETS ${INSTALL_PYTHON_MODULES}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
      )
endif()

install(TARGETS ${INSTALL_MODULES}
    EXPORT ${PROJECT_NAME}_Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Export target configuration (for installation)
install(EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

# Export target configuration (for local building)
export(EXPORT ${PROJECT_NAME}_Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  )

if(ADD_MODULE_COLLISION)
  install(
      DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/collision/include/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()

if(ADD_MODULE_GEOMETRY)
  install(
      DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/geometry/include/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif()

if(BUILD_DOC)
    if(NOT (ADD_PYTHON_BINDINGS AND ADD_MODULE_COLLISION AND ADD_MODULE_GEOMETRY))
        message(FATAL_ERROR "BUILD_DOC requires ADD_PYTHON_BINDINGS, ADD_MODULE_COLLISION and ADD_MODULE_GEOMETRY")
    endif()
    add_subdirectory(../doc doc)
endif()
