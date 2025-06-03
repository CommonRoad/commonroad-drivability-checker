#ifndef COLLISION_FCL_FCL_COLLISION_OBJECT_H_
#define COLLISION_FCL_FCL_COLLISION_OBJECT_H_

#include <Eigen/Dense>

#include "collision/collision_object.h"
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/solvers/fcl/fcl_entity_type.h"
#include "collision/solvers/fcl/i_fcl_collision_object.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"

#include <atomic>
#include <memory>

namespace collision {
namespace solvers {
namespace solverFCL {
class FCLCollisionObject : public SolverEntity_FCL {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FCLCollisionObject(const CollisionObject *parent,
                     const IFCLCollisionObject *parent_fcl_interface) {
    m_parent = parent;
    m_parent_fcl_interface = parent_fcl_interface;
    invaldateCollisionObjectCache();
  }

  FCLCollisionObject(const FCLCollisionObject &copy) {
    m_parent = copy.m_parent;
    m_parent_fcl_interface = copy.m_parent_fcl_interface;
    invaldateCollisionObjectCache();
  };

  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const {
    return FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT;
  }

  virtual ~FCLCollisionObject() {}

  std::shared_ptr<fcl::CollisionObject<FCL_PRECISION>> getCollisionObject_fcl(
      void) const;

  int calculateDistance(const CollisionObject &obj2, FCL_PRECISION &distance,
                        FCL_PRECISION tolerance = 1e-6) const;
  int calculateDistanceNegTolerance(
      const CollisionObject &obj2, double &distance,
      FCL_TOLERANCE_CHECK_TYPE check_type = TOLERANCE_CHECK_NARROWPHASE,
      FCL_PRECISION tolerance = 1e-6) const;

  virtual bool BVCheck(CollisionObjectConstPtr obj2) const;

  virtual collision::RectangleAABBConstPtr getAABB(void) const;

  virtual void invalidateSolverEntityCache(void) const override {
    invaldateCollisionObjectCache();
  }

  const CollisionObject *getParent(void) const { return m_parent; }

 protected:
  inline void invaldateCollisionObjectCache(void) const {
    object_cached = false;
  }

 private:
  mutable bool object_cached;
  mutable std::shared_ptr<fcl::CollisionObject<FCL_PRECISION>> fcl_col_obj;
  mutable std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> fcl_geometry;
  const CollisionObject *m_parent;
  const IFCLCollisionObject *m_parent_fcl_interface;
};

typedef std::shared_ptr<const FCLCollisionObject> FCLCollisionObjectConstPtr;

}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision

#endif
