if(ADD_MODULE_COLLISION)
include_directories("./collision/include")
pybind11_add_module(pycrcc SHARED src/module_collision.cc src/py_bind.cc)
set_target_properties(pycrcc PROPERTIES COMPILE_DEFINITIONS "PY_WRAPPER_MODULE_COLLISION=1")

target_link_libraries(pycrcc PRIVATE crcc)

if(NOT S11N_LIBRARY)
	message(STATUS "Serialization and pickling support have been disabled for the Python library")
	set(S11N_LIBRARY "")
	add_definitions(-DENABLE_SERIALIZER=0)
endif()
endif()

if(ADD_MODULE_GEOMETRY)
pybind11_add_module(pycrccosy src/module_geometry.cc src/py_bind.cc)

set_target_properties(pycrccosy PROPERTIES COMPILE_DEFINITIONS "PY_WRAPPER_MODULE_GEOMETRY=1 ")

target_link_libraries(pycrccosy PRIVATE crccosy)

message(STATUS "${PYTHON_INCLUDE_DIR}")
endif()
