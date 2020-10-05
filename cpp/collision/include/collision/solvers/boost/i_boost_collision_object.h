#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_BOOST_COLLISION_OBJECT_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_BOOST_COLLISION_OBJECT_H_

#include "collision/solvers/boost/i_solver_entity_boost.h"

namespace collision {
namespace solvers {
namespace solverBoost {
class IBoostCollisionObject : public ISolverEntity_Boost {
 public:
  virtual BOOST_COLLISION_ENTITY_TYPE getBoostEntityType(void) const override {
    return BOOST_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_BOOST_OBJECT;
  }
};
}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_BOOST_COLLISION_OBJECT_H_ \
        */
