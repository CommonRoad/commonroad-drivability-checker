#include <iostream>
#include "collision_tests.h"

int main() {
  std::cout << "TEST STARTED" << std::endl;

  while (true) {
    int ret = test::test_polygon();
    if (ret != 0) {
      std::cout << "TEST FAILED" << std::endl;
      exit(1);
    }
  }
  return 0;
}
