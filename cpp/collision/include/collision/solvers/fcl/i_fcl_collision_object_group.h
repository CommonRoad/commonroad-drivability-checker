#pragma once
#include <fcl/narrowphase/collision_object.h>
#include <vector>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"

namespace collision {
namespace solvers {
namespace solverFCL {
class IFCLCollisionObjectGroup : public ISolverEntity_FCL {
 public:
  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const override {
    return FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP;
  }

  virtual int getCollisionObjects(
      std::vector<fcl::CollisionObject<FCL_PRECISION> *> &) const = 0;
  virtual int elementCount(void) const = 0;
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
