#pragma once
#include "collision/collision_object.h"
#include "collision/solvers/distance_requests.h"

namespace collision {
std::size_t distance(const CollisionObject &obj1, const CollisionObject &obj2,
                     DistanceResult &res, const DistanceRequest &req);
}
