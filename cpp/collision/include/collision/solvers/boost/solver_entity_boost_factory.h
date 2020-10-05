#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_FACTORY_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_FACTORY_H_
#include "collision/collision_object_ex.h"
#include "collision/solvers/boost/boost_collision_object.h"
#include "collision/solvers/boost/i_boost_collision_object.h"
#include "collision/solvers/boost/i_solver_entity_boost.h"
#include "collision/solvers/boost/solver_entity_boost.h"

namespace collision {
namespace solvers {
namespace solverBoost {
SolverEntity_Boost *createBoostSolverEntity(const CollisionObjectEx *obj) {
  const ISolverEntity_Boost *entity = obj->getBoostInterface();
  if (!entity) return nullptr;
  switch (entity->getBoostEntityType()) {
    case COLLISION_ENTITY_TYPE_BOOST_OBJECT:
      return new BoostCollisionObject(
          static_cast<const CollisionObject *>(obj));
      break;
    default:
      return nullptr;
      break;
  }
}
}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_FACTORY_H_ \
        */
