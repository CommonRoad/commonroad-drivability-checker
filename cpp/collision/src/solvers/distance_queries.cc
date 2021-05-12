#include "collision/solvers/distance_requests.h"
#include "collision/solvers/fcl/fcl_helpers.h"

#include "collision/solvers/distance_queries.h"

namespace collision {
namespace detail {
std::size_t inline distance_default(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    DistanceResult &res,
                                    const DistanceRequest &req) {
  SolverEntity_FCL *obj1_entity;
  FCL_COLLISION_ENTITY_TYPE obj2_entity_type =
      get_object_fcl_entity_type(&obj1, obj1_entity);

  if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    double distance = 0;
    int ret =
        (static_cast<FCLCollisionObject *>(obj1_entity))
            ->calculateDistance(obj2, distance, req.computation_tolerance);
    if (!ret) {
      res.setMinDistance(distance);
    }
    return ret;

  } else if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
    double distance = 0;
    int ret =
        (static_cast<FCLCollisionObjectGroup *>(obj1_entity))
            ->calculateDistance(obj2, distance, req.computation_tolerance);
    if (!ret) {
      res.setMinDistance(distance);
    }
    return ret;
  } else {
    return -1;
  }
}

std::size_t inline distance_tolerance_neg(const CollisionObject &obj1,
                                          const CollisionObject &obj2,
                                          DistanceResult &res,
                                          const DistanceRequest &req) {
  SolverEntity_FCL *obj1_entity;
  FCL_COLLISION_ENTITY_TYPE obj2_entity_type =
      get_object_fcl_entity_type(&obj1, obj1_entity);

  if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    double distance = 0;
    int ret =
        (static_cast<FCLCollisionObject *>(obj1_entity))
            ->calculateDistanceNegTolerance(obj2, distance,
                                            req.dist_node_type == DNT_AABB
                                                ? TOLERANCE_CHECK_BB
                                                : TOLERANCE_CHECK_NARROWPHASE,
                                            req.computation_tolerance);

    if (ret >= 0) {
      res.setMinDistance(distance);
      if (distance > -1 * (abs(req.compare_tolerance)))
        res.setTolerancePassed(true);
      else
        res.setTolerancePassed(false);
    }
    return ret;

  } else if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
    double distance = 0;
    int ret =
        (static_cast<FCLCollisionObjectGroup *>(obj1_entity))
            ->calculateDistanceNegTolerance(obj2, distance,
                                            req.dist_node_type == DNT_AABB
                                                ? TOLERANCE_CHECK_BB
                                                : TOLERANCE_CHECK_NARROWPHASE,
                                            req.computation_tolerance);

    if (ret >= 0) {
      res.setMinDistance(distance);
      if (distance > -1 * (abs(req.compare_tolerance)))
        res.setTolerancePassed(true);
      else
        res.setTolerancePassed(false);
    }
    return ret;
  } else {
    return -1;
  }
}
}  // namespace detail

// Only DRT_TOLERANCE_NEG works at the moment. It is implemented in FCL for some
// shapes. For test purposes only. Underlying implementations of distance queries
// in FCL had bugs.

std::size_t distance(const CollisionObject &obj1, const CollisionObject &obj2,
                     DistanceResult &res, const DistanceRequest &req) {
  if (req.dist_request_type == DRT_DISTANCE) {
    return detail::distance_default(obj1, obj2, res, req);
  } else if (req.dist_request_type == DRT_TOLERANCE_NEG) {
    return detail::distance_tolerance_neg(obj1, obj2, res, req);
  } else
    return -1;
}

}  // namespace collision
