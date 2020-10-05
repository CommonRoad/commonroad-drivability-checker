#pragma once
#include "collision/collision_object_ex.h"
#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/fcl_entity_type.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"

namespace collision {
namespace solvers {
namespace solverFCL {
inline FCL_COLLISION_ENTITY_TYPE get_object_fcl_entity_type(
    const CollisionObject *obj2) {
  const CollisionObjectEx *obj = static_cast<const CollisionObjectEx *>(obj2);
  if (!obj) return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  SolverEntity_FCL *req_entity;
  obj->getSolverEntity(req_entity);
  if (!req_entity) {
    if (obj->getCollisionObjectClass() ==
        CollisionObjectClass::OBJ_CLASS_TVOBSTACLE) {
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_TVOBJECT;
    } else
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  } else
    return req_entity->getFclEntityType();
}

inline const FCLCollisionObjectGroup *get_fcl_object_group_ptr(
    const CollisionObject *obj) {
  const CollisionObjectEx *obj_ex = static_cast<const CollisionObjectEx *>(obj);
  if (!obj_ex) return nullptr;
  SolverEntity_FCL *fcl_entity;
  obj_ex->getSolverEntity(fcl_entity);
  if (fcl_entity &&
      fcl_entity->getFclEntityType() == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
    return static_cast<const FCLCollisionObjectGroup *>(fcl_entity);
  } else
    return nullptr;
}

inline const FCLCollisionObject *get_fcl_object_ptr(
    const CollisionObject *obj) {
  const CollisionObjectEx *obj_ex = static_cast<const CollisionObjectEx *>(obj);
  if (!obj_ex) return nullptr;
  SolverEntity_FCL *fcl_entity;
  obj_ex->getSolverEntity(fcl_entity);

  if (fcl_entity &&
      fcl_entity->getFclEntityType() == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return static_cast<const FCLCollisionObject *>(fcl_entity);
  } else
    return nullptr;
}

inline FCL_COLLISION_ENTITY_TYPE get_object_fcl_entity_type(
    CollisionObjectConstPtr obj2, SolverEntity_FCL *&req_entity) {
  const CollisionObjectEx *obj =
      static_cast<const CollisionObjectEx *>(obj2.get());
  if (!obj) return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  obj->getSolverEntity(req_entity);
  if (!req_entity) {
    if (obj->getCollisionObjectClass() ==
        CollisionObjectClass::OBJ_CLASS_TVOBSTACLE) {
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_TVOBJECT;
    } else
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  } else
    return req_entity->getFclEntityType();
}

inline FCL_COLLISION_ENTITY_TYPE get_object_fcl_entity_type(
    const CollisionObject *obj2, SolverEntity_FCL *&req_entity) {
  const CollisionObjectEx *obj = static_cast<const CollisionObjectEx *>(obj2);
  if (!obj) return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  obj->getSolverEntity(req_entity);
  if (!req_entity) {
    if (obj->getCollisionObjectClass() ==
        CollisionObjectClass::OBJ_CLASS_TVOBSTACLE) {
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_TVOBJECT;
    } else
      return FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_INVALID;
  } else
    return req_entity->getFclEntityType();
}
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
