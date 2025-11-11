#include <iostream>
#include "collision_tests.h"

int main() {
  int ret = test::test_polygon();
  if (ret != 0) {
    std::cout << "TEST FAILED" << std::endl;
  } else {
    std::cout << "TEST PASSED" << std::endl;
  }
  return ret;
}
