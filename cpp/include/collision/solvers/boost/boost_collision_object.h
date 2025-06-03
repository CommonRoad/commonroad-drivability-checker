#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_OBJECT_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_OBJECT_H_

#include <Eigen/Dense>
#include <atomic>
#include <memory>

#include "collision/collision_object.h"
#include "collision/solvers/boost/boost_entity_type.h"
#include "collision/solvers/boost/boost_object_internal.h"
#include "collision/solvers/boost/solver_entity_boost.h"

namespace collision {
namespace solvers {
namespace solverBoost {
class BoostCollisionObject : public SolverEntity_Boost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BoostCollisionObject(const CollisionObject *parent) {
    m_parent = parent;
    invaldateCollisionObjectCache();
  }

  BoostCollisionObject(const BoostCollisionObject &copy) {
    m_parent = copy.m_parent;
    invaldateCollisionObjectCache();
  };

  virtual BOOST_COLLISION_ENTITY_TYPE getBoostEntityType(void) const {
    return BOOST_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_BOOST_OBJECT;
  }

  virtual ~BoostCollisionObject() {}

  std::shared_ptr<BoostObjectInternal> getCollisionObject_boost(void) const;

  virtual void invalidateSolverEntityCache(void) const override {
    invaldateCollisionObjectCache();
  }

  const CollisionObject *getParent(void) const { return m_parent; }

 protected:
  const CollisionObject *m_parent;
  inline void invaldateCollisionObjectCache(void) const {
    object_cached = false;
  }

 private:
  mutable std::shared_ptr<BoostObjectInternal> boost_obj;
  mutable bool object_cached;
};

typedef std::shared_ptr<const BoostCollisionObject>
    BoostCollisionObjectConstPtr;

}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_OBJECT_H_ \
        */
