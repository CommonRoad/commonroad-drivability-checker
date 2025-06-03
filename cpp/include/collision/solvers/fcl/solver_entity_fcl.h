#pragma once

#include <memory>
#include "collision/solvers/fcl/fcl_entity_type.h"
namespace collision {
namespace solvers {
namespace solverFCL {
class SolverEntity_FCL {
 public:
  virtual void invalidateSolverEntityCache(void) const = 0;

  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const {
    return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_UNKNOWN;
  }

  virtual ~SolverEntity_FCL(){};
};

typedef std::shared_ptr<SolverEntity_FCL> FCLSolverEntityPtr;
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
