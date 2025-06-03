#include "collision/application.h"
#if ENABLE_COLLISION_TESTS
#include "collision/tests/test_common.h"

#include <fstream>

namespace collision {
namespace test {

template <typename T>
std::ofstream TestFailureLogger<T>::out_file;
template <typename T>
std::mutex TestFailureLogger<T>::file_mutex;
template <typename T>
int TestFailureLogger<T>::num_copies = 0;

template <typename T>
TestFailureLogger<T>::TestFailureLogger(void) {
  static bool notified_of_file_open_error = false;
  std::lock_guard<std::mutex> guard(file_mutex);
  if (!num_copies) {
    out_file.open(get_filename(), std::ofstream::out | std::ofstream::app);
    if (out_file.fail())
      if (!notified_of_file_open_error) {
        std::cout << "Failed to open error log file" << std::endl;
        notified_of_file_open_error = true;
      }
  }
  num_copies++;
}

template <typename T>
TestFailureLogger<T>::~TestFailureLogger(void) {
  std::lock_guard<std::mutex> guard(file_mutex);
  num_copies--;
  if (!num_copies && out_file.is_open()) {
    out_file.close();
  }
}

template <typename T>
int TestFailureLogger<T>::log_failure(std::string log_str) {
  std::lock_guard<std::mutex> guard(file_mutex);
  if (out_file.is_open() && !out_file.fail()) {
    out_file << log_str;
  }
  return 0;
}

template class TestFailureLogger<TestFailureLoggerNarrowphase>;
template class TestFailureLogger<TestFailureLoggerBroadphase>;
template class TestFailureLogger<TestFailureLoggerParentMap>;

template <>
const char *TestFailureLogger<TestFailureLoggerNarrowphase>::get_filename(
    void) {
  static const char narrowphase_filename[] = NARROWPHASE_FILENAME;
  return narrowphase_filename;
}
template <>
const char *TestFailureLogger<TestFailureLoggerBroadphase>::get_filename(void) {
  static const char broadphase_filename[] = BROADPHASE_FILENAME;
  return broadphase_filename;
}

template <>
const char *TestFailureLogger<TestFailureLoggerParentMap>::get_filename(void) {
  static const char parentmap_filename[] = PARENTMAP_FILENAME;
  return parentmap_filename;
}

}  // namespace test
}  // namespace collision

#endif
