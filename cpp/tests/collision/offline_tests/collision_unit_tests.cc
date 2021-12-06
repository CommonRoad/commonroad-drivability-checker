#include <iostream>
#include "collision_tests.h"

int main() {
  int ret = 0;
  if ((ret = test::test_polygon())) {
    std::cout << "TEST FAILED" << std::endl;
  } else {
    std::cout << "TEST PASSED" << std::endl;
  }
  return 0;
}
