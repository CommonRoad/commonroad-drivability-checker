#include "collision/solvers/fcl/performance_timers.h"
#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace test {

#if (TIME_PROFILE_ENABLED)
constexpr int TIMER_COUNT = 20;

std::vector<unsigned long> perfdata_0;
unsigned long perf_data[TIMER_COUNT];
uint64_t call_count[TIMER_COUNT];
Timer perf_timers[TIMER_COUNT];
std::string timer_messages[TIMER_COUNT];
bool timers_started[TIMER_COUNT];
bool timers_enabled = false;
#endif

void start_timer(int index) {
#if (TIME_PROFILE_ENABLED)
  if (timers_enabled) {
    // if(timers_started[index])
    //{
    //	perf_timers[index].resume();
    //}
    // else
    //{
    // timers_started[index]=true;
    perf_timers[index].start();

    //}
  }
#endif
}
void stop_timer(int index) {
#if (TIME_PROFILE_ENABLED)
  if (timers_enabled) {
    perf_timers[index].stop();
    perf_data[index] += perf_timers[index].getElapsedTimeInNanoSec();
    call_count[index]++;
  }
#endif
}

int init_perfomance_timers() {
#if (TIME_PROFILE_ENABLED)

  timer_messages[0] = "windowQuery elapsed time: ";
  timer_messages[1] = "timeSlice elapsed time: ";
  timer_messages[2] = "collide elapsed time: ";
  timer_messages[3] = "collide_3 elapsed time: ";
  timer_messages[4] = "union elapsed time: ";
  timer_messages[5] = "chull elapsed time: ";
  timer_messages[6] = "TIMER_boost_obb_obb_convex_hull: ";
  timer_messages[7] = "TIMER_get_cand: ";
  timer_messages[8] = "TIMER_difference :";
  timer_messages[9] = "GRID_BUILD :";
  timer_messages[10] = "GRID_CANDIDATES :";
  timer_messages[11] = "GRID_NARROWPHASE :";
  timer_messages[12] = "GRID_STATIC_TOTAL :";
  timer_messages[13] = "GRID_HASHING :";
  timer_messages[14] = "grid_total_body :";
  timer_messages[15] = "TIMER_poly_build :";
  timer_messages[16] = "TIMER_grid_window_query :";
  timer_messages[17] = "TIMER_polygon_enclosure :";

  for (int i = 0; i < TIMER_COUNT; i++) {
    perf_data[i] = 0;
    call_count[i] = 0;
  }
  timers_enabled = true;
#else
  volatile int a = 0;
  asm volatile("nop");
#endif

  return 0;
}

int report_perfomance() {
#if (TIME_PROFILE_ENABLED)

  timers_enabled = false;
  // for(int i=0; i<10;i++)
  //{
  //	perf_timers[0].stop();
  //}
  for (int i = 0; i < TIMER_COUNT; i++) {
    std::cout << timer_messages[i] << " " << int(perf_data[i] / 1000) << " "
              << call_count[i] << "\n";
  }
#else
  volatile int a = 0;
  asm volatile("nop");

#endif

  return 0;
}

#if (TIME_PROFILE_ENABLED)
//==============================================================================
Timer::Timer() {
#ifdef _WIN32
  QueryPerformanceFrequency(&frequency);
  startCount.QuadPart = 0;
  endCount.QuadPart = 0;
#else
  startCount.tv_sec = startCount.tv_usec = 0;
  endCount.tv_sec = endCount.tv_usec = 0;
#endif

  stopped = 0;
  startTimeInMicroSec = 0;
  endTimeInMicroSec = 0;
}

//==============================================================================
Timer::~Timer() {
  // Do nothing
}

//==============================================================================
void Timer::start() {
  stopped = 0;  // reset stop flag
#ifdef _WIN32
  QueryPerformanceCounter(&startCount);
#else
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  // gettimeofday(&startCount, nullptr);
#endif
}

//==============================================================================
void Timer::stop() {
  stopped = 1;  // set timer stopped flag

#ifdef _WIN32
  QueryPerformanceCounter(&endCount);
#else
  clock_gettime(CLOCK_MONOTONIC, &time_end);
  // gettimeofday(&endCount, nullptr);
#endif
}
unsigned long Timer::getElapsedTimeInNanoSec() {
  if (!stopped) clock_gettime(CLOCK_MONOTONIC, &time_end);
  timespec time_diff;
  time_diff = diff(time_start, time_end);

  if (time_diff.tv_sec) {
    return time_diff.tv_sec * 10000000000 + time_diff.tv_nsec;
  } else {
    return time_diff.tv_nsec;
  }
}
double Timer::getElapsedTimeInMicroSec() {
#ifdef _WIN32
  if (!stopped) QueryPerformanceCounter(&endCount);

  startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
  endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
  if (!stopped) clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time_end);
    // gettimeofday(&endCount, nullptr);

    // startTimeInMicroSec = (startCount.tv_sec * 1000000.0) +
    // startCount.tv_usec; endTimeInMicroSec = (endCount.tv_sec * 1000000.0) +
    // endCount.tv_usec;
#endif
  timespec time_diff;
  time_diff = diff(time_start, time_end);

  if (time_diff.tv_sec) {
    return time_diff.tv_sec * 10000000000 + time_diff.tv_nsec / 1000;
  } else {
    return time_diff.tv_nsec / 1000;
  }
}

//==============================================================================
double Timer::getElapsedTimeInMilliSec() {
  return this->getElapsedTimeInMicroSec() * 0.001;
}

//==============================================================================
double Timer::getElapsedTimeInSec() {
  return this->getElapsedTimeInMicroSec() * 0.000001;
}

//==============================================================================
double Timer::getElapsedTime() { return this->getElapsedTimeInMilliSec(); }

timespec diff(timespec start, timespec end) {
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}
#endif

}  // namespace test
