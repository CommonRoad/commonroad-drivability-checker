
#pragma once
#include "collision/collision_object.h"
#include "collision/i_collision_container.h"
#include "collision/solvers/boost/i_solver_entity_boost.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"

namespace collision {

using namespace solvers::solverFCL;
using namespace solvers::solverBoost;

namespace solvers {
namespace solverFCL {
class SolverEntity_FCL;
struct SolverEntity_FCLDeleter {
  void operator()(SolverEntity_FCL *p);
};
}  // namespace solverFCL
}  // namespace solvers

namespace solvers {
namespace solverBoost {
class SolverEntity_Boost;
struct SolverEntity_BoostDeleter {
  void operator()(SolverEntity_Boost *p);
};
}  // namespace solverBoost
}  // namespace solvers

/*!
\brief Provides functionality to use different collision solvers

*/
class CollisionObjectEx : public collision::CollisionObject {
 public:
  CollisionObjectEx() {
    fcl_solver_entity_valid_ = false;
    boost_solver_entity_valid_ = false;
  }

  virtual bool collide(
      const CollisionObject &c,
      const collision::CollisionRequest &req = CollisionRequest()) const;

  virtual bool BVCheck(CollisionObjectConstPtr obj2) const;

  virtual std::shared_ptr<const collision::RectangleAABB> getAABB() const;

  virtual int getSolverEntity(solvers::solverFCL::SolverEntity_FCL *&ptr) const;
  virtual int getSolverEntity(
      solvers::solverBoost::SolverEntity_Boost *&ptr) const;

  virtual const ICollisionContainer *getContainerInterface(void) const {
    return nullptr;
  }

  virtual const solvers::solverFCL::ISolverEntity_FCL *getFclInterface(
      void) const {
    return nullptr;
  }

  virtual const solvers::solverBoost::ISolverEntity_Boost *getBoostInterface(
      void) const {
    return nullptr;
  }

 protected:
  void invalidateCollisionEntityCache(void);

 private:
  mutable std::unique_ptr<solvers::solverFCL::SolverEntity_FCL,
                          solvers::solverFCL::SolverEntity_FCLDeleter>
      fcl_entity_;
  mutable bool fcl_solver_entity_valid_;

  mutable std::unique_ptr<solvers::solverBoost::SolverEntity_Boost,
                          solvers::solverBoost::SolverEntity_BoostDeleter>
      boost_entity_;
  mutable bool boost_solver_entity_valid_;
};
}  // namespace collision
