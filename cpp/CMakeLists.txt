# CONF
project(DrivabilityChecker)
cmake_minimum_required(VERSION 3.12)

option(BUILD_SHARED_LIBS "Build s11n as a shared library" OFF)

option(ADD_TESTS "unit tests" OFF)
option(BUILD_DOC "generate the doc target." OFF)
option(ADD_MODULE_GEOMETRY "add geometry module" ON)
option(ADD_MODULE_COLLISION "add collision module" ON)
option(ADD_PYTHON_BINDINGS "add Python bindings" OFF)

# Add modules for discovering Sphinx etc.
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../CMakeModules" ${CMAKE_MODULE_PATH})

if(ADD_PYTHON_BINDINGS)
    
	message(STATUS ${PYTHON_INCLUDE_DIR})
	message(STATUS ${PYTHON_LIBRARY})
	message(STATUS ${PYTHON_EXECUTABLE})
	
	
	if((NOT PYTHON_INCLUDE_DIR) OR (NOT PYTHON_LIBRARY) OR (NOT PYTHON_EXECUTABLE))
		if((NOT PYTHON_VERSION) OR (NOT PATH_TO_PYTHON_ENVIRONMENT)) 
			message( FATAL_ERROR "Python not found" )
		endif()
		if (${PYTHON_VERSION} GREATER_EQUAL 3.8)
			set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VERSION})
			set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VERSION}.so)
			set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VERSION})
		else()
			set(PYTHON_INCLUDE_DIR ${PATH_TO_PYTHON_ENVIRONMENT}/include/python${PYTHON_VERSION}m)
			set(PYTHON_LIBRARY ${PATH_TO_PYTHON_ENVIRONMENT}/lib/libpython${PYTHON_VERSION}m.so)
			set(PYTHON_EXECUTABLE ${PATH_TO_PYTHON_ENVIRONMENT}/bin/python${PYTHON_VERSION}m)
		endif()
	endif()
    #set(Python3_ROOT "$ENV{CONDA_PREFIX}")
	#message(STATUS "python dir: ${Python3_ROOT}")
    #find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development REQUIRED)
	#set 

    # Specific binary directory required here
    add_subdirectory(../third_party/pybind11 third_party/pybind11)
endif()

### Shared configuration

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Discover dependencies

find_package(s11n)

find_package(Eigen3 3.0.5 QUIET CONFIG)

# If Eigen3Config.cmake is not found, use the FindEigen3.cmake module
if(NOT Eigen3_FOUND)
  find_package(Eigen3 3.0.5 QUIET MODULE)
  set(Eigen3_FOUND ON)
endif()

if(Eigen3_FOUND)
  set(COM_HAVE_EIGEN TRUE)
else()
  message(SEND_ERROR "EIGEN3 (>= 3.0.5) is required by the CollisionChecker")
  set(COM_HAVE_EIGEN FALSE)
endif()

find_package(OpenMP)

# Targets/Python modules to be installed
set(INSTALL_MODULES "")
set(INSTALL_PYTHON_MODULES "")


# Provides configure_package_config_file
include(CMakePackageConfigHelpers)

# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

# Static library for triangle sources

add_library(triangle STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c/triangle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c/triangle.h")

target_compile_definitions(triangle PRIVATE -DANSI_DECLARATORS=1 -DTRILIBRARY=1 -DREAL=double -DVOID=int)

set_property(TARGET triangle PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(triangle
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/triangle/c
    )



### Definition of crcc

if(ADD_MODULE_COLLISION)
    add_subdirectory(collision)

    set(INSTALL_MODULES ${INSTALL_MODULES} crcc)
    set(INSTALL_PYTHON_MODULES ${INSTALL_PYTHON_MODULES} pycrcc crcc)
endif(ADD_MODULE_COLLISION)

### Definition of crccosy

if(ADD_MODULE_GEOMETRY)
    add_subdirectory(geometry)

    set(INSTALL_MODULES ${INSTALL_MODULES} crccosy)
    set(INSTALL_PYTHON_MODULES ${INSTALL_PYTHON_MODULES} pycrccosy crccosy)
endif(ADD_MODULE_GEOMETRY)

if(ADD_PYTHON_BINDINGS)
    add_subdirectory(python_binding)
endif()

if(ADD_TESTS)
    add_subdirectory(tests)
endif()

if(BUILD_DOC)
    if(NOT (ADD_PYTHON_BINDINGS AND ADD_MODULE_COLLISION AND ADD_MODULE_GEOMETRY))
        message(FATAL_ERROR "BUILD_DOC requires ADD_PYTHON_BINDINGS, ADD_MODULE_COLLISION and ADD_MODULE_GEOMETRY")
    endif()
    add_subdirectory(../doc doc)
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
