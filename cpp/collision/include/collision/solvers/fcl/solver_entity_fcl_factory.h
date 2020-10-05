#pragma once

#include "collision/collision_object_ex.h"
#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/i_fcl_collision_object.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"

namespace collision {
namespace solvers {
namespace solverFCL {
SolverEntity_FCL *createFCLSolverEntity(const CollisionObjectEx *obj) {
  const ISolverEntity_FCL *entity = obj->getFclInterface();
  if (!entity) return nullptr;
  switch (entity->getFclEntityType()) {
    case COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP:
      return new FCLCollisionObjectGroup(
          obj->getContainerInterface(), obj,
          static_cast<const IFCLCollisionObjectGroup *>(entity));
      break;
    case COLLISION_ENTITY_TYPE_FCL_OBJECT:
      return new FCLCollisionObject(
          static_cast<const CollisionObject *>(obj),
          static_cast<const IFCLCollisionObject *>(entity));
      break;
    default:
      return nullptr;
      break;
  }
}
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
