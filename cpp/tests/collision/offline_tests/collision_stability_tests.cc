#include <iostream>
#include "collision_tests.h"

int main() {
  std::cout << "TEST STARTED" << std::endl;

  while (true) {
    int ret = 0;
    if ((ret = test::test_polygon())) {
      std::cout << "TEST FAILED" << std::endl;
      exit(1);
    } else {
      // std::cout << "TEST PASSED" << std::endl;
    }
  }
  return 0;
}
