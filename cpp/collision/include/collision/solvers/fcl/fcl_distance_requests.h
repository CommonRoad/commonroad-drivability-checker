#pragma once
#include <fcl/narrowphase/distance.h>
#include "collision/solvers/fcl/fcl_collision_object.h"

namespace collision {
namespace solvers {
namespace solverFCL {
double penetrationDepth(fcl::AABBd boxA, fcl::AABBd boxB);
bool defaultDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                             fcl::CollisionObject<FCL_PRECISION> *o2,
                             void *cdata_, FCL_PRECISION &dist);
bool toleranceDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                               fcl::CollisionObject<FCL_PRECISION> *o2,
                               void *cdata_, FCL_PRECISION &dist);
bool toleranceBBDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                                 fcl::CollisionObject<FCL_PRECISION> *o2,
                                 void *cdata_, FCL_PRECISION &dist);

FCL_PRECISION get_max_distance(void);
struct DistanceDataBase {
  DistanceDataBase(void);

  FCL_PRECISION minDist;
  fcl::DistanceRequest<FCL_PRECISION> request;

  fcl::DistanceResult<FCL_PRECISION> result;

  bool done;
};

struct DistanceData : public DistanceDataBase {
  DistanceData(void);
  int result_code;
};

struct ToleranceDistanceData : public DistanceData {
  ToleranceDistanceData(void);
  FCL_PRECISION tolerance;
};
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision
