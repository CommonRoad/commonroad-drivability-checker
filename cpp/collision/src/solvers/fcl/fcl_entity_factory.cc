#include "collision/shape_group.h"
#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/i_fcl_collision_object.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"

namespace collision {
namespace solvers {
namespace solverFCL {
SolverEntity_FCL *createFCLSolverEntity(CollisionObject *obj,
                                        ISolverEntity_FCL *entity) {
  switch (entity->getFclEntityType()) {
    case COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP:
      return new FCLCollisionObjectGroup(
          (static_cast<CollisionObjectEx *>(obj))->getContainerInterface(), obj,
          static_cast<IFCLCollisionObjectGroup *>(entity));
      break;
    case COLLISION_ENTITY_TYPE_FCL_OBJECT:
      return new FCLCollisionObject(obj,
                                    static_cast<IFCLCollisionObject *>(entity));
      break;
    default:
      return nullptr;
      break;
  }
}
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
