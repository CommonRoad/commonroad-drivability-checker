#pragma once
#define RAYTRACE_DEBUG 0

#ifndef ENABLE_SERIALIZER
#define ENABLE_SERIALIZER 1
#endif

#if ENABLE_SERIALIZER
#define SERIALIZER_DEFAULT_FORMAT "compact"
#endif

#ifndef ENABLE_TRIANGULATION
#define ENABLE_TRIANGULATION 1
#endif

#ifndef ENABLE_GPC
#define ENABLE_GPC 1
#endif

#ifndef ENABLE_POLYGON_VALIDITY_CHECKS
#define ENABLE_POLYGON_VALIDITY_CHECKS 0
#endif

#ifndef ENABLE_COLLISION_TESTS
#define ENABLE_COLLISION_TESTS 0
#endif

#if ENABLE_COLLISION_TESTS

#ifndef ENABLE_SERIALIZER_TESTS
#define ENABLE_SERIALIZER_TESTS 0
#endif

#ifndef TOLERANCE_BBONLY
#define TOLERANCE_BBONLY 1
#endif

#ifndef ENABLE_COLLISION_TESTS_NARROWPHASE

#define ENABLE_COLLISION_TESTS_NARROWPHASE 0
#define POLYGON_BRUTEFORCE 1
#endif

namespace collision {
namespace test {

#ifndef PARENTMAP_FILENAME
#define PARENTMAP_FILENAME "/tmp/log_parentmap.txt"
#endif
#ifndef BROADPHASE_FILENAME
#define BROADPHASE_FILENAME "/tmp/log_broadphase.txt"
#endif
#ifndef BROADPHASE_DUMP_DIRECTORY
#define BROADPHASE_DUMP_DIRECTORY "/tmp/dumps"
#endif
#ifndef NARROWPHASE_DUMP_DIRECTORY
#define NARROWPHASE_DUMP_DIRECTORY "/tmp/dumps"
#endif
#ifndef NARROWPHASE_FILENAME
#define NARROWPHASE_FILENAME "/tmp/log_narrowphase.txt"
#endif
}  // namespace test
}  // namespace collision
#endif
