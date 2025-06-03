set(python_module_name pycrcc)
set(install_destination commonroad_dc)
nanobind_add_module(${python_module_name} MODULE
        pybind.h
        pybind.cc
        pybind_shapes.h
        pybind_shapes.cc
        pybind_util.h
        pybind_util.cc
)
target_link_libraries(${python_module_name} PRIVATE crcc)

# link libs11n
if (CR_DC_BUILD_S11N)
    message(STATUS "<${PROJECT_NAME}> Serialization and pickling support have been ENABLED for ${python_module_name}")
    target_link_libraries(${python_module_name} PUBLIC s11n::s11n)
    target_compile_definitions(${python_module_name} PRIVATE -DENABLE_SERIALIZER=1)
else ()
    message(STATUS "<${PROJECT_NAME}> Serialization and pickling support have been DISABLED for ${python_module_name}")
    target_compile_definitions(${python_module_name} PRIVATE -DENABLE_SERIALIZER=0)
endif ()

install(TARGETS ${python_module_name}
        LIBRARY DESTINATION ${install_destination}
        COMPONENT ${python_module_name}
)

add_custom_target(install-python-modules-${python_module_name}
        ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_COMPONENT=${python_module_name}
        -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
        DEPENDS ${python_module_name}
)

target_precompile_headers(nanobind-static
        PUBLIC
        <nanobind/nanobind.h>
)

if (SKBUILD)
    # only add Python typing stubs if we are building with scikit-build, otherwise, Python dependencies might be missing
    nanobind_add_stub(
            ${python_module_name}_stub
            MODULE ${python_module_name}
            OUTPUT ${python_module_name}.pyi
            PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
            DEPENDS ${python_module_name}
            MARKER_FILE py.typed
            VERBOSE
    )

    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${python_module_name}.pyi
            ${CMAKE_CURRENT_BINARY_DIR}/py.typed
            DESTINATION ${install_destination}
            COMPONENT PythonModules
    )

    # Sanity check: Ensure we are building all dependencies as static libraries
    include(utils/EnsureStatic)
    if (SKBUILD_SELF_CONTAINED)
        # If we are building all dependencies ourselves, then they should
        # all be static.
        ensure_all_static(crcc)
    else ()
        # If we are partially using system libraries, then only the crcc library
        # itself has to be static.
        ensure_static(crcc)
    endif ()
endif ()
