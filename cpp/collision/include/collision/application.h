#ifndef CPP_COLLISION_APPLICATION_H_
#define CPP_COLLISION_APPLICATION_H_

#include "collision/application_settings.h"

#include "collision/solvers/fcl/performance_timers.h"

#if TIME_PROFILE_ENABLED
#define STACK_TIMER ::test::StackTimer
#else
#define STACK_TIMER
#endif

#if ENABLE_COLLISION_TESTS == 1

#include "collision/tests/collision_tests.h"

#endif

static void tim(int a) {
  // placeholder
}

#define TIMER_windowQuery 0
#define TIMER_timeSlice 1
#define TIMER_collide 2
#define TIMER_collide_3 3
#define TIMER_union 4
#define TIMER_chull_polygon 5
#define TIMER_boost_obb_obb_convex_hull 6
#define TIMER_get_cand 7
#define TIMER_difference 8
#define TIMER_grid_build 9
#define TIMER_grid_candidates 10
#define TIMER_grid_narrowphase 11
#define TIMER_grid_static_total 12
#define TIMER_grid_hashing 13
#define TIMER_grid_total_body 14
#define TIMER_poly_build 15
#define TIMER_grid_window_query 16
#define TIMER_polygon_enclosure 17

#if TIME_PROFILE_ENABLED
#define TIMER_START(x) ::test::start_timer(x);
#define TIMER_STOP(x)  ::test::stop_timer(x);
#else
#define TIMER_START(x)
#define TIMER_STOP(x)
#endif

#endif /* CPP_COLLISION_APPLICATION_H_ */
