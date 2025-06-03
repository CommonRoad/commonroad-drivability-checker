#pragma once
#include <stdlib.h>
#include "collision/collision_object.h"
#include "collision/collision_object_types.h"
#include "collision/solvers/collision_requests.h"

namespace collision {
namespace solvers {
class PrimitiveSolver {};
class FCLSolver {};
class DefaultSolver {};

typedef std::size_t (*collide_bool_func_t)(const CollisionObject &obj1,
                                           const CollisionObject &obj2,
                                           CollisionResult &res,
                                           const CollisionRequest &req);

class CollisionFunctionMatrix {
 public:
  // CollisionFunctionMatrix(const solvers::PrimitiveSolver* solver);
  CollisionFunctionMatrix(const solvers::FCLSolver *solver);
  CollisionFunctionMatrix(const solvers::DefaultSolver *solver);
  collide_bool_func_t getSolverBoolFunction(
      CollisionObjectType obj1_type, CollisionObjectType obj2_type) const {
    return m_collide_bool_function[obj1_type][obj2_type];
  }

 private:
  collide_bool_func_t m_collide_bool_function[COL_OBJECT_TYPES_COUNT]
                                             [COL_OBJECT_TYPES_COUNT];
};
}  // namespace solvers
}  // namespace collision
