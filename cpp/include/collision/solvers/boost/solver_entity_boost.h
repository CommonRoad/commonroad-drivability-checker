#include "boost_entity_type.h"

#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_H_

namespace collision {
namespace solvers {
namespace solverBoost {

class SolverEntity_Boost {
 public:
  virtual void invalidateSolverEntityCache(void) const = 0;

  virtual BOOST_COLLISION_ENTITY_TYPE getBoostEntityType(void) const {
    return BOOST_COLLISION_ENTITY_TYPE::BOOST_COLLISION_ENTITY_TYPE_UNKNOWN;
  }

  virtual ~SolverEntity_Boost(){};
};
}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_SOLVER_ENTITY_BOOST_H_ \
        */
