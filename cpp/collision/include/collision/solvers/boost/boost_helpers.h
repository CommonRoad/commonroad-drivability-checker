#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_HELPERS_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_HELPERS_H_

#include "collision/collision_object_ex.h"
#include "collision/solvers/boost/boost_collision_object.h"

namespace collision {
namespace solvers {
namespace solverBoost {
inline const BoostCollisionObject *get_boost_object_ptr(
    const CollisionObject *obj) {
  const CollisionObjectEx *obj_ex = static_cast<const CollisionObjectEx *>(obj);
  if (!obj_ex) return nullptr;
  SolverEntity_Boost *boost_entity;
  obj_ex->getSolverEntity(boost_entity);

  if (boost_entity && boost_entity->getBoostEntityType() ==
                          COLLISION_ENTITY_TYPE_BOOST_OBJECT) {
    return static_cast<const BoostCollisionObject *>(boost_entity);
  } else
    return nullptr;
}
}  // namespace solverBoost
}  // namespace solvers

}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_HELPERS_H_ */
