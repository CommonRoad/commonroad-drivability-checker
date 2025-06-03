#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_SOLVER_ENTITY_BOOST_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_SOLVER_ENTITY_BOOST_H_

#pragma once

#include <memory>

#include "collision/solvers/boost/boost_entity_type.h"
namespace collision {
namespace solvers {
namespace solverBoost {
class ISolverEntity_Boost {
 public:
  virtual BOOST_COLLISION_ENTITY_TYPE getBoostEntityType(void) const {
    return BOOST_COLLISION_ENTITY_TYPE_UNKNOWN;
  }
  virtual ~ISolverEntity_Boost(void) {}
};
}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_I_SOLVER_ENTITY_BOOST_H_ \
        */
