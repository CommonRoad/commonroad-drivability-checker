if(ADD_MODULE_COLLISION)

pybind11_add_module(pycrcc src/module_collision.cc src/py_bind.cc)

target_compile_definitions(pycrcc PRIVATE -DPY_WRAPPER_MODULE_COLLISION=1)

target_link_libraries(pycrcc PRIVATE crcc_obj)

if(NOT s11n_FOUND)
	message(STATUS "Serialization and pickling support have been disabled for the Python library")
	target_compile_definitions(pycrcc PRIVATE -DENABLE_SERIALIZER=0)
endif()

endif(ADD_MODULE_COLLISION)

if(ADD_MODULE_GEOMETRY)
pybind11_add_module(pycrccosy src/module_geometry.cc src/py_bind.cc)

target_compile_definitions(pycrccosy PRIVATE -DPY_WRAPPER_MODULE_GEOMETRY=1)

target_link_libraries(pycrccosy PRIVATE crccosy_obj)

message(STATUS "${PYTHON_INCLUDE_DIR}")
endif(ADD_MODULE_GEOMETRY)
