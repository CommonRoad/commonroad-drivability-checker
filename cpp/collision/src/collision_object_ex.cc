#include "collision/collision_object_ex.h"

#include "collision/solvers/boost/solver_entity_boost_factory.h"
#include "collision/solvers/collision_queries.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl_factory.h"

namespace collision {

bool CollisionObjectEx::collide(const CollisionObject &c,
                                const collision::CollisionRequest &req) const {
  CollisionResult res;
  collision::collide_binary(*this, c, res, req);
  return res.collides();
}

bool CollisionObjectEx::BVCheck(CollisionObjectConstPtr obj2) const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->BVCheck(obj2);
  }
  return true;
}

std::shared_ptr<const collision::RectangleAABB> CollisionObjectEx::getAABB()
    const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->getAABB();
  }
  std::cout
      << "Returning bounding volume for complex shapes is not implemented";
  throw;
}

void CollisionObjectEx::invalidateCollisionEntityCache(void) {
  if (fcl_solver_entity_valid_) {
    fcl_entity_->invalidateSolverEntityCache();
  }
  if (boost_solver_entity_valid_) {
    boost_entity_->invalidateSolverEntityCache();
  }
}
int CollisionObjectEx::getSolverEntity(SolverEntity_FCL *&ptr) const {
  SolverEntity_FCL *cur_value = fcl_entity_.get();
  if (fcl_solver_entity_valid_) {
    ptr = cur_value;
    return 0;
  } else {
    // FCL
    fcl_entity_ =
        std::move(std::unique_ptr<SolverEntity_FCL, SolverEntity_FCLDeleter>(
            createFCLSolverEntity(this)));
    fcl_solver_entity_valid_ = true;
    ptr = fcl_entity_.get();
    return 0;
  }
}
int CollisionObjectEx::getSolverEntity(SolverEntity_Boost *&ptr) const {
  SolverEntity_Boost *cur_value = boost_entity_.get();
  if (boost_solver_entity_valid_) {
    ptr = cur_value;
    return 0;
  } else {
    // Boost
    boost_entity_ = std::move(
        std::unique_ptr<SolverEntity_Boost, SolverEntity_BoostDeleter>(
            createBoostSolverEntity(this)));
    boost_solver_entity_valid_ = true;
    ptr = boost_entity_.get();
    return 0;
  }
}

}  // namespace collision
