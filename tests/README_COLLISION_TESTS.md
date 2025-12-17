How to run collision checker random tests:

1. Run the script run_collision_tests.py, make sure it returns 0 as exit code.
2. Run the script collision/collision_random_tests.py
   Monitor the memory usage using the top command. Make sure that the reserved memory for the python process is not growing.
2. Build collision checker C++ library:
	$ cmake -DCMAKE_BUILD_TYPE=Release -B build-cmake -S .
	$ cmake --build build-cmake -j JOB_COUNT
   In the build-cmake folder there are two C++ tests:
    cpp_unit_tests
    cpp_stability_tests.
   Run cpp_unit_tests to run Boost Polygon unit tests.
   Run cpp_stability_tests and monitor for memory leaks as described in (2). The program runs indefinitely and needs to be stopped with CNTRL+C when one is sure that the memory usage is not growing.
    
