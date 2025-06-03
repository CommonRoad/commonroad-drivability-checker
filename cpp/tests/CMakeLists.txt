#//project (cpp_tests)
add_executable(cpp_unit_tests
        offline_tests/collision_unit_tests.cc
        offline_tests/collision_tests.cc
)
add_executable(cpp_stability_tests
        offline_tests/collision_stability_tests.cc
        offline_tests/collision_tests.cc
)

target_link_libraries(cpp_unit_tests PRIVATE crcc)
target_link_libraries(cpp_stability_tests PRIVATE crcc)
