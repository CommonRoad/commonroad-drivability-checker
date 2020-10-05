#pragma once

#include <stdlib.h>
#include "collision/collision_object.h"
#include "collision/collision_object_types.h"
#include "collision/solvers/collision_solvers.h"

namespace collision {
namespace solvers {

// warning: not all of the functions are implemented

namespace solverPrimitive {
std::size_t collide_aabb_aabb(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req);
std::size_t collide_aabb_obb(const CollisionObject &obj1,
                             const CollisionObject &obj2, CollisionResult &res,
                             const CollisionRequest &req);
std::size_t collide_aabb_sphere(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req);
std::size_t collide_aabb_triangle(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req);
std::size_t collide_aabb_point(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);

std::size_t collide_obb_aabb(const CollisionObject &obj1,
                             const CollisionObject &obj2, CollisionResult &res,
                             const CollisionRequest &req);
std::size_t collide_obb_obb(const CollisionObject &obj1,
                            const CollisionObject &obj2, CollisionResult &res,
                            const CollisionRequest &req);
std::size_t collide_obb_sphere(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);
std::size_t collide_obb_triangle(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req);
std::size_t collide_obb_point(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req);

std::size_t collide_sphere_aabb(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req);
std::size_t collide_sphere_obb(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);
std::size_t collide_sphere_sphere(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req);
std::size_t collide_sphere_triangle(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req);
std::size_t collide_sphere_point(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req);

std::size_t collide_triangle_aabb(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req);
std::size_t collide_triangle_obb(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req);
std::size_t collide_triangle_sphere(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req);
std::size_t collide_triangle_triangle(const CollisionObject &obj1,
                                      const CollisionObject &obj2,
                                      CollisionResult &res,
                                      const CollisionRequest &req);
std::size_t collide_triangle_point(const CollisionObject &obj1,
                                   const CollisionObject &obj2,
                                   CollisionResult &res,
                                   const CollisionRequest &req);

std::size_t collide_point_aabb(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req);
std::size_t collide_point_obb(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req);
std::size_t collide_point_sphere(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req);
std::size_t collide_point_triangle(const CollisionObject &obj1,
                                   const CollisionObject &obj2,
                                   CollisionResult &res,
                                   const CollisionRequest &req);
std::size_t collide_point_point(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req);

std::size_t collide_polygon_obj(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req);
std::size_t collide_polygon_polygon(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req);
std::size_t collide_obj_polygon(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
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
}  // namespace solverPrimitive
}  // namespace solvers
}  // namespace collision
