#include "collision/collision_object.h"

#if ENABLE_SERIALIZER
#include "collision/serialize/public/serialize_public.h"
#endif

#include "collision/solvers/collision_queries.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl_factory.h"

namespace collision {

#if ENABLE_SERIALIZER
int CollisionObject::serialize(std::ostream &output_stream) const {
  return serialize::serialize(*this, output_stream);
}

CollisionObjectConstPtr CollisionObject::deserialize(
    std::istream &input_stream) {
  CollisionObjectConstPtr ret;
  if (!serialize::deserialize(ret, input_stream)) {
    return ret;
  } else
    return CollisionObjectConstPtr(0);
}
#endif

void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(shared_from_this());
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(shared_from_this());
    parent_map.emplace(this, newlist);
  }
}
void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map,
    CollisionObjectConstPtr parent) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(parent);
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(parent);
    parent_map.emplace(this, newlist);
  }
}

bool CollisionObject::collide(const CollisionObject &c,
                                const collision::CollisionRequest &req) const {
  CollisionResult res;
  collision::collide_binary(*this, c, res, req);
  return res.collides();
}

bool CollisionObject::BVCheck(CollisionObjectConstPtr obj2) const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->BVCheck(obj2);
  }
  return true;
}

std::shared_ptr<const collision::RectangleAABB> CollisionObject::getAABB()
    const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity && entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->getAABB();
  }
  std::cout
      << "Returning bounding volume for complex shapes is not implemented";
  throw;
}

void CollisionObject::invalidateCollisionEntityCache(void) {
  if (fcl_solver_entity_valid_) {
    fcl_entity_->invalidateSolverEntityCache();
  }
}
int CollisionObject::getSolverEntity(SolverEntity_FCL *&ptr) const {
  SolverEntity_FCL *cur_value = fcl_entity_.get();
  if (fcl_solver_entity_valid_) {
    ptr = cur_value;
    return 0;
  } else {
    // FCL
    fcl_entity_ =
        std::move(std::unique_ptr<SolverEntity_FCL>(
            createFCLSolverEntity(this)));
    fcl_solver_entity_valid_ = true;
    ptr = fcl_entity_.get();
    return 0;
  }
}
}  // namespace collision
