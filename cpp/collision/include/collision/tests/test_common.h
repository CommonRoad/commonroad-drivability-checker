#ifndef TESTS_COLLISION_TEST_COMMON_H_
#define TESTS_COLLISION_TEST_COMMON_H_

#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

namespace collision {
namespace test {

class ITestFailureLogger {
 public:
  virtual int log_failure(std::string log_str) = 0;
  virtual ~ITestFailureLogger() {}
};

template <class T>
class TestFailureLogger : public ITestFailureLogger {
 public:
  TestFailureLogger(void);

  int log_failure(std::string log_str);
  virtual ~TestFailureLogger(void);

 private:
  static const char *get_filename(void);

  static int num_copies;
  static std::ofstream out_file;
  static std::mutex file_mutex;
};

class TestFailureLoggerParentMap
    : public TestFailureLogger<TestFailureLoggerParentMap> {};
class TestFailureLoggerBroadphase
    : public TestFailureLogger<TestFailureLoggerBroadphase> {};
class TestFailureLoggerNarrowphase
    : public TestFailureLogger<TestFailureLoggerNarrowphase> {};

template <>
const char *TestFailureLogger<TestFailureLoggerBroadphase>::get_filename(void);
template <>
const char *TestFailureLogger<TestFailureLoggerNarrowphase>::get_filename(void);
template <>
const char *TestFailureLogger<TestFailureLoggerParentMap>::get_filename(void);

}  // namespace test

}  // namespace collision

#endif /* TESTS_COLLISION_TEST_COMMON_H_ */
