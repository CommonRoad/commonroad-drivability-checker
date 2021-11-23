set(PYBIND_SRC_ROOT ${PYBIND_ROOT}/src)

if(ADD_MODULE_COLLISION)

   set(PYBIND_SRC_COLLISION 
        ${PYBIND_SRC_ROOT}/module_collision.cc
        ${PYBIND_SRC_ROOT}/py_bind.cc
       )

    pybind11_add_module(pycrcc ${PYBIND_SRC_COLLISION})

    target_compile_definitions(pycrcc PRIVATE -DPY_WRAPPER_MODULE_COLLISION=1)

    target_link_libraries(pycrcc PUBLIC crcc)

if(s11n_FOUND)
    target_compile_definitions(pycrcc PRIVATE -DENABLE_SERIALIZER=1)
    target_link_libraries(pycrcc PUBLIC s11n::s11n)
else()
    target_compile_definitions(pycrcc PRIVATE -DENABLE_SERIALIZER=0)
endif()




    # $ORIGIN is a special value: It represents the location of the shared object
    # being loaded itself. By setting the RPATH of the installed Python module
    # to $ORIGIN, the loader will automatically look for shared libraries in the
    # same directory as the module file itself.
    #
    # This is required here so that the loader finds e.g. libcrcc.so which is
    # installed next to the Python module, even if the Python module got moved
    # during installation.
    if(APPLE)
          set_property(TARGET pycrcc PROPERTY INSTALL_RPATH "@loader_path")
    endif()
    #set_property(TARGET pycrcc PROPERTY INSTALL_RPATH "{$ORIGIN}")

    if(NOT s11n_FOUND)
        message(STATUS "Serialization and pickling support have been disabled for the Python library")
        target_compile_definitions(pycrcc PRIVATE -DENABLE_SERIALIZER=0)
    endif()

    if(APPLE)
         target_include_directories(pycrcc PRIVATE "/usr/local/include")
    endif(APPLE)


endif(ADD_MODULE_COLLISION)

if(ADD_MODULE_GEOMETRY)

    set(PYBIND_SRC_GEOMETRY
        ${PYBIND_SRC_ROOT}/module_geometry.cc
        ${PYBIND_SRC_ROOT}/py_bind.cc
       )

    pybind11_add_module(pycrccosy ${PYBIND_SRC_GEOMETRY})

    target_compile_definitions(pycrccosy PRIVATE -DPY_WRAPPER_MODULE_GEOMETRY=1)

#if (OPENMP_FOUND)
    if(APPLE)
         target_link_directories(pycrccosy PUBLIC "/usr/local/lib")
    endif(APPLE)
    target_link_libraries(pycrccosy PUBLIC OpenMP::OpenMP_CXX)
    if(APPLE)
         target_include_directories(pycrccosy PRIVATE "/usr/local/include")
    endif(APPLE)

#endif(OPENMP_FOUND)

    target_link_libraries(pycrccosy PUBLIC crccosy)

if(s11n_FOUND)
    target_compile_definitions(pycrccosy PRIVATE -DENABLE_SERIALIZER=1)
    target_link_libraries(pycrccosy PUBLIC s11n::s11n)
else()
    target_compile_definitions(pycrccosy PRIVATE -DENABLE_SERIALIZER=0)
endif()


    # See note on $ORIGIN above
    if(APPLE)
          set_property(TARGET pycrccosy PROPERTY INSTALL_RPATH "@loader_path")
    endif()
    message(STATUS "${PYTHON_INCLUDE_DIR}")
endif(ADD_MODULE_GEOMETRY)
