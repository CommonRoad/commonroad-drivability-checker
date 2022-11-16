#ifndef CPP_COLLISION_FCL_PERFOMANCE_TIMERS_H_
#define CPP_COLLISION_FCL_PERFOMANCE_TIMERS_H_

#define TIME_PROFILE_ENABLED (1)

#if TIME_PROFILE_ENABLED
#include <sys/time.h>
#ifdef _WIN32
#include <windows.h>
#endif

//#include <boost/timer.hpp>

namespace test {

int init_perfomance_timers();

int report_perfomance();

void start_timer(int index);
void stop_timer(int index);

class StackTimer {
 public:
  StackTimer(int index) : m_index_(index) {
#if TIME_PROFILE_ENABLED
    start_timer(index);
#endif
  }
  ~StackTimer() {
#if TIME_PROFILE_ENABLED
    stop_timer(m_index_);
#endif
  }

 protected:
  int m_index_ = -1;
};
timespec diff(timespec start, timespec end);

class Timer {
 public:
  Timer();
  ~Timer();

  void start();                       ///< start timer
  void stop();                        ///< stop the timer
  double getElapsedTime();            ///< get elapsed time in milli-second
  double getElapsedTimeInSec();       ///< get elapsed time in second (same as
                                      ///< getElapsedTime)
  double getElapsedTimeInMilliSec();  ///< get elapsed time in milli-second
  double getElapsedTimeInMicroSec();  ///< get elapsed time in micro-second
  unsigned long
  getElapsedTimeInNanoSec();  ///< get elapsed time in micro-second

 private:
  timespec time_start;
  timespec time_end;

  double startTimeInMicroSec;  ///< starting time in micro-second
  double endTimeInMicroSec;    ///< ending time in micro-second
  int stopped;                 ///< stop flag
#ifdef _WIN32
  LARGE_INTEGER frequency;  ///< ticks per second
  LARGE_INTEGER startCount;
  LARGE_INTEGER endCount;
#else
  timeval startCount;
  timeval endCount;
#endif
};
}  // namespace test
#endif

#endif /* CPP_COLLISION_FCL_PERFOMANCE_TIMERS_H_ */
