#pragma once

#include "collision/solvers/fcl/fcl_entity_type.h"

#include <memory>
namespace collision {
namespace solvers {
namespace solverFCL {
class ISolverEntity_FCL {
 public:
  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const {
    return FCL_COLLISION_ENTITY_TYPE_UNKNOWN;
  }
  virtual ~ISolverEntity_FCL(void) {}
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
