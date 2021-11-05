if(ADD_MODULE_COLLISION)
    pybind11_add_module(pycrcc src/module_collision.cc src/py_bind.cc)

    target_compile_definitions(pycrcc PRIVATE -DPY_WRAPPER_MODULE_COLLISION=1)

    target_link_libraries(pycrcc PRIVATE crcc)

    # $ORIGIN is a special value: It represents the location of the shared object
    # being loaded itself. By setting the RPATH of the installed Python module
    # to $ORIGIN, the loader will automatically look for shared libraries in the
    # same directory as the module file itself.
    #
    # This is required here so that the loader finds e.g. libcrcc.so which is
    # installed next to the Python module, even if the Python module got moved
    # during installation.

    set_property(TARGET pycrcc PROPERTY INSTALL_RPATH "$ORIGIN")

    if(NOT s11n_FOUND)
        message(STATUS "Serialization and pickling support have been disabled for the Python library")
        target_compile_definitions(pycrcc PRIVATE -DENABLE_SERIALIZER=0)
    endif()
endif(ADD_MODULE_COLLISION)

if(ADD_MODULE_GEOMETRY)
    pybind11_add_module(pycrccosy src/module_geometry.cc src/py_bind.cc)

    target_compile_definitions(pycrccosy PRIVATE -DPY_WRAPPER_MODULE_GEOMETRY=1)

    target_link_libraries(pycrccosy PRIVATE crccosy)

    # See note on $ORIGIN above

    set_property(TARGET pycrccosy PROPERTY INSTALL_RPATH "$ORIGIN")

    message(STATUS "${PYTHON_INCLUDE_DIR}")
endif(ADD_MODULE_GEOMETRY)
