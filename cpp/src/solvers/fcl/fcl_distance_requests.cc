#include "collision/solvers/fcl/fcl_distance_requests.h"
#include <stdlib.h>

namespace collision {
namespace solvers {
namespace solverFCL {
bool defaultDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                             fcl::CollisionObject<FCL_PRECISION> *o2,
                             void *cdata_, FCL_PRECISION &dist) {
  DistanceData *cdata = static_cast<DistanceData *>(cdata_);
  const fcl::DistanceRequest<FCL_PRECISION> &request = cdata->request;
  fcl::DistanceResult<FCL_PRECISION> &result = cdata->result;
  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }
  if (o1->collisionGeometry() == o2->collisionGeometry()) {
    dist = get_max_distance();
    cdata->result_code = -1;
  } else {
    // dist = -1 * penetrationDepth(o1->getAABB(), o2->getAABB());
    fcl::distance(o1, o2, request, result);
    dist = result.min_distance;
  }
  if (cdata->minDist > dist) {
    cdata->minDist = dist;
  }
  if (dist <= 0) return true;  // in collision or in touch
  return cdata->done;
}
double penetrationDepth(fcl::AABBd boxA, fcl::AABBd boxB) {
  typedef double S;
  S result = 0;
  if (boxA.overlap(boxB)) {
    for (std::size_t i = 0; i < 3; ++i) {
      const S &amin = boxA.min_[i];
      const S &amax = boxA.max_[i];
      const S &bmin = boxB.min_[i];
      const S &bmax = boxB.max_[i];

      if (amin < bmax) {
        S delta = bmax - amin;
        result += delta * delta;
      } else if (amax > bmin) {
        S delta = amax - bmin;
        result += delta * delta;
      }
    }
  }
  return std::sqrt(result);
}

bool toleranceBBDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                                 fcl::CollisionObject<FCL_PRECISION> *o2,
                                 void *cdata_, FCL_PRECISION &dist) {
  ToleranceDistanceData *cdata = static_cast<ToleranceDistanceData *>(cdata_);
  const fcl::DistanceRequest<FCL_PRECISION> &request = cdata->request;

  fcl::DistanceResult<FCL_PRECISION> &result = cdata->result;
  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }
  // TODO: add a check that geometries are not the same
  if (o1->collisionGeometry() == o2->collisionGeometry()) {
    dist = get_max_distance();
    cdata->result_code = -1;
  } else {
    dist = -1 * penetrationDepth(o1->getAABB(), o2->getAABB());
  }

  if (cdata->minDist > dist) {
    cdata->minDist = dist;
  }
  if (dist < -abs(cdata->tolerance))
    return true;  // one object penetrates more than tolerated
  return cdata->done;
}

bool toleranceDistanceFunction(fcl::CollisionObject<FCL_PRECISION> *o1,
                               fcl::CollisionObject<FCL_PRECISION> *o2,
                               void *cdata_, FCL_PRECISION &dist) {
  ToleranceDistanceData *cdata = static_cast<ToleranceDistanceData *>(cdata_);
  const fcl::DistanceRequest<FCL_PRECISION> &request = cdata->request;

  fcl::DistanceResult<FCL_PRECISION> &result = cdata->result;
  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }
  // TODO: add a check that geometries are not the same
  if (o1->collisionGeometry() == o2->collisionGeometry()) {
    dist = get_max_distance();
    cdata->result_code = -1;
  } else {
    fcl::distance(o1, o2, request, result);
    dist = result.min_distance;
  }

  if (cdata->minDist > dist) {
    cdata->minDist = dist;
  }
  if (dist < -abs(cdata->tolerance))
    return true;  // one object penetrates more than tolerated
  return cdata->done;
}
FCL_PRECISION get_max_distance(void) {
  return std::numeric_limits<FCL_PRECISION>::max();
}
DistanceDataBase::DistanceDataBase(void) {
  done = false;
  minDist = get_max_distance();
}
DistanceData::DistanceData(void) { this->result_code = 0; }
ToleranceDistanceData::ToleranceDistanceData(void) {
  tolerance = DEFAULT_DISTANCE_TOLERANCE;
}
}  // namespace solverFCL

}  // namespace solvers
}  // namespace collision
