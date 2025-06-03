#pragma once
#include "collision/collision_object.h"
#include "collision/solvers/collision_requests.h"

namespace collision {

std::size_t collide_binary(const CollisionObject& obj1,
                           const CollisionObject& obj2, CollisionResult& res,
                           const CollisionRequest& req);

}  // namespace collision
