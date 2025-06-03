#pragma once
#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"

namespace collision {
namespace solvers {
namespace solverFCL {
namespace fcl_primitive_queries {
int FCL_CalculateDistance(const FCLCollisionObject &obj1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance,
                          double distance_tolerance = 1e-6);

int FCL_CalculateDistance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance,
                          double distance_tolerance = 1e-6);
int FCL_CalculateDistance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObjectGroup &group2,
                          FCL_PRECISION &distance,
                          double distance_tolerance = 1e-6);

int FCL_DistanceTolerance(
    const FCLCollisionObjectGroup &group1, const FCLCollisionObject &obj2,
    FCL_PRECISION &distance,
    FCL_TOLERANCE_CHECK_TYPE check_type = TOLERANCE_CHECK_NARROWPHASE,
    double distance_tolerance = 1e-6);
int FCL_DistanceTolerance(
    const FCLCollisionObjectGroup &group1,
    const FCLCollisionObjectGroup &group2, FCL_PRECISION &distance,
    FCL_TOLERANCE_CHECK_TYPE check_type = TOLERANCE_CHECK_NARROWPHASE,
    double distance_tolerance = 1e-6);
int FCL_DistanceTolerance(
    const FCLCollisionObject &obj1, const FCLCollisionObject &obj2,
    FCL_PRECISION &distance,
    FCL_TOLERANCE_CHECK_TYPE check_type = TOLERANCE_CHECK_NARROWPHASE,
    double distance_tolerance = 1e-6);
}  // namespace fcl_primitive_queries
}  // namespace solverFCL

}  // namespace solvers

}  // namespace collision
