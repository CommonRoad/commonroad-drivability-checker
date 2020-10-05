#pragma once
#include "collision/collision_object.h"
#include "collision/collision_object_types.h"
#include "collision/solvers/collision_solvers.h"

namespace collision {

namespace solvers {
namespace solverFCL {

std::size_t collide_obj_obj(const CollisionObject &obj1,
                            const CollisionObject &obj2, CollisionResult &res,
                            const CollisionRequest &req);
std::size_t collide_shape_group_obj(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req);
std::size_t collide_obj_shape_group(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req);

std::size_t collide_shape_group_shape_group(const CollisionObject &obj1,
                                            const CollisionObject &obj2,
                                            CollisionResult &res,
                                            const CollisionRequest &req);

std::size_t collide_tvobst_obj(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);
std::size_t collide_obj_tvobst(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);
std::size_t collide_tvobst_tvobst(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req);

}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
