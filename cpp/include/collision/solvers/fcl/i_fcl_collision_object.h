#pragma once
#include <memory>
#include "collision/solvers/fcl/i_solver_entity_fcl.h"

#include "collision/solvers/fcl/fcl_decl.h"

namespace fcl {
template <typename T>
class CollisionGeometry;
template <typename T>
class CollisionObject;
}  // namespace fcl

namespace collision {
namespace solvers {
namespace solverFCL {
class IFCLCollisionObject : public ISolverEntity_FCL {
 public:
  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const override {
    return FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT;
  }

  virtual fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const = 0;
  virtual fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &) const = 0;
};
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision
