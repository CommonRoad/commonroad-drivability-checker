#include "collision/solvers/fcl/fcl_distance_queries.h"
#include "collision/solvers/fcl/fcl_distance_requests.h"
//#include "collision/solverFCL/fcl_collision_object.h"
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/distance.h>
namespace collision {
namespace solvers {
namespace solverFCL {
namespace fcl_primitive_queries {
int FCL_CalculateDistance(const FCLCollisionObject &obj1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance, double distance_tolerance) {
  fcl::DistanceRequest<FCL_PRECISION> request;
  request.gjk_solver_type = fcl::GST_LIBCCD;
  request.distance_tolerance = distance_tolerance;
  fcl::DistanceResult<FCL_PRECISION> result;
  fcl::CollisionObject<FCL_PRECISION> *fcl_obj1 =
      obj1.getCollisionObject_fcl().get();
  fcl::CollisionObject<FCL_PRECISION> *fcl_obj2 =
      obj2.getCollisionObject_fcl().get();
  if (!fcl_obj1 || !fcl_obj2) {
    return -1;
  }
  FCL_PRECISION retval = 1;

  if (fcl_obj1->collisionGeometry() == fcl_obj2->collisionGeometry()) {
    retval = -2;
    distance = get_max_distance();
  } else {
    retval = fcl::distance(fcl_obj1, fcl_obj2, request, result);

    distance = result.min_distance;
  }

  return int(retval);
}
int FCL_DistanceTolerance(const FCLCollisionObject &obj1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance,
                          FCL_TOLERANCE_CHECK_TYPE check_type,
                          double distance_tolerance) {
  fcl::DistanceRequest<FCL_PRECISION> request;
  request.distance_tolerance = distance_tolerance;
  request.enable_signed_distance = true;
  request.enable_nearest_points = false;
  request.gjk_solver_type = fcl::GST_LIBCCD;
  fcl::DistanceResult<FCL_PRECISION> dist_result;
  fcl::CollisionObject<FCL_PRECISION> *fcl_obj1 =
      obj1.getCollisionObject_fcl().get();
  fcl::CollisionObject<FCL_PRECISION> *fcl_obj2 =
      obj2.getCollisionObject_fcl().get();
  if (!fcl_obj1 || !fcl_obj2) {
    return -1;
  }
  if (fcl_obj1->collisionGeometry() == fcl_obj2->collisionGeometry()) {
    return -2;
    distance = get_max_distance();
  } else {
    if (check_type == FCL_TOLERANCE_CHECK_TYPE::TOLERANCE_CHECK_NARROWPHASE) {
      FCL_PRECISION retval =
          fcl::distance(fcl_obj1, fcl_obj2, request, dist_result);

      distance = dist_result.min_distance;
    } else {
      distance =
          -1 * penetrationDepth(fcl_obj1->getAABB(), fcl_obj2->getAABB());
    }
  }

  if (distance == get_max_distance()) {
    return 1;
  } else
    return 0;
}
int FCL_CalculateDistance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance, double distance_tolerance) {
  DistanceData distData;
  fcl::BroadPhaseCollisionManager<double> *mngr;
  distData.request.gjk_solver_type = fcl::GST_LIBCCD;
  group1.getManager_fcl(mngr);
  if (!mngr) return -1;
  fcl::CollisionObject<FCL_PRECISION> *obj2_ptr =
      obj2.getCollisionObject_fcl().get();
  if (!obj2_ptr) {
    return -1;
  }
  mngr->distance(obj2_ptr, &distData, defaultDistanceFunction);
  if (distData.result_code == -1) {
    return -2;  // Group contained object with the same Geometry as obj2
  }
  if (distData.minDist == get_max_distance()) {
    return 1;
  } else {
    distance = distData.minDist;
    return 0;
  }
}
int FCL_CalculateDistance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObjectGroup &group2,
                          FCL_PRECISION &distance, double distance_tolerance) {
  DistanceData distData;
  fcl::BroadPhaseCollisionManager<double> *mngr1;
  fcl::BroadPhaseCollisionManager<double> *mngr2;
  distData.request.gjk_solver_type = fcl::GST_LIBCCD;
  group1.getManager_fcl(mngr1);
  if (!mngr1) return -1;
  group2.getManager_fcl(mngr2);
  if (!mngr2) return -1;
  mngr1->distance(mngr2, &distData, defaultDistanceFunction);
  if (distData.result_code == -1) {
    return -2;  // Group contained object with the same Geometry as obj2
  }
  if (distData.minDist == get_max_distance()) {
    return 1;
  } else {
    distance = distData.minDist;
    return 0;
  }
}

int FCL_DistanceTolerance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObject &obj2,
                          FCL_PRECISION &distance,
                          FCL_TOLERANCE_CHECK_TYPE check_type,
                          double distance_tolerance) {
  ToleranceDistanceData distData;
  distData.tolerance = distance_tolerance;
  distData.request.distance_tolerance = distance_tolerance;
  distData.request.enable_signed_distance = true;
  distData.request.enable_nearest_points = false;
  distData.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  fcl::BroadPhaseCollisionManager<double> *mngr;
  group1.getManager_fcl(mngr);
  if (!mngr) return -1;
  fcl::CollisionObject<FCL_PRECISION> *obj2_ptr =
      obj2.getCollisionObject_fcl().get();
  if (!obj2_ptr) {
    return -1;
  }
  if (check_type == FCL_TOLERANCE_CHECK_TYPE::TOLERANCE_CHECK_NARROWPHASE) {
    mngr->distance(obj2_ptr, &distData, toleranceDistanceFunction);
  } else {
    mngr->distance(obj2_ptr, &distData, toleranceBBDistanceFunction);
  }

  if (distData.result_code == -1) {
    return -2;  // Group contained object with the same Geometry as obj2
  }
  if (distData.minDist == get_max_distance()) {
    return 1;
  } else {
    distance = distData.minDist;
    return 0;
  }
}
int FCL_DistanceTolerance(const FCLCollisionObjectGroup &group1,
                          const FCLCollisionObjectGroup &group2,
                          FCL_PRECISION &distance,
                          FCL_TOLERANCE_CHECK_TYPE check_type,
                          double distance_tolerance) {
  ToleranceDistanceData distData;
  distData.tolerance = distance_tolerance;
  distData.request.distance_tolerance = distance_tolerance;
  distData.request.enable_signed_distance = true;
  distData.request.enable_nearest_points = false;
  distData.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  fcl::BroadPhaseCollisionManager<double> *mngr1;
  fcl::BroadPhaseCollisionManager<double> *mngr2;
  group1.getManager_fcl(mngr1);
  if (!mngr1) return -1;
  group2.getManager_fcl(mngr2);
  if (!mngr2) return -1;
  if (check_type == FCL_TOLERANCE_CHECK_TYPE::TOLERANCE_CHECK_NARROWPHASE) {
    mngr1->distance(mngr2, &distData, toleranceDistanceFunction);
  } else {
    mngr1->distance(mngr2, &distData, toleranceBBDistanceFunction);
  }

  if (distData.result_code == -1) {
    return -2;  // Group contained object with the same Geometry as obj2
  }
  if (distData.minDist == get_max_distance()) {
    return 1;
  } else {
    distance = distData.minDist;
    return 0;
  }
}
}  // namespace fcl_primitive_queries
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision
