#ifndef COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_
#define COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_

#include <fcl/narrowphase/collision_object.h>
#include <Eigen/Dense>
#include "collision/solvers/fcl/fcl_broadphase_manager_factory.h"
#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_decl.h"

#include "collision/i_collision_container.h"
#include "collision/solvers/fcl/i_fcl_collision_object_group.h"

#include <vector>

#include "collision/solvers/fcl/fcl_entity_type.h"

namespace collision {
namespace solvers {
namespace solverFCL {
class FCLCollisionObjectGroup : public SolverEntity_FCL {
 public:
  FCLCollisionObjectGroup(const ICollisionContainer *, const CollisionObject *,
                          const IFCLCollisionObjectGroup *parent_interface);

  FCLCollisionObjectGroup(const FCLCollisionObjectGroup &copy);

  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const {
    return FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP;
  }

  int replaceBroadphaseFactory(
      BroadPhaseManagerFactoryConstPtr broadphase_factory);

  int calculateDistance(const CollisionObject &obj2, FCL_PRECISION &distance,
                        FCL_PRECISION tolerance = 1e-6) const;
  int calculateDistanceNegTolerance(
      const CollisionObject &obj2, double &distance,
      FCL_TOLERANCE_CHECK_TYPE check_type = TOLERANCE_CHECK_NARROWPHASE,
      FCL_PRECISION tolerance = 1e-6) const;

  virtual ~FCLCollisionObjectGroup(void) {}

  int getManager_fcl(fcl::BroadPhaseCollisionManager<FCL_PRECISION> *&)
      const;  // TODO: create and/or cache an appropriate broadphase manager

  const ICollisionContainer *getContainer(void) const {
    return m_CollisionContainer;
  }

  const IFCLCollisionObjectGroup *getParentInterface(void) const {
    return m_parent_interface;
  }

  const CollisionObject *getParent(void) const { return m_parent; }

  virtual void invalidateSolverEntityCache(void) const override {
    invalidateManagerCache();
  }

 protected:
  void invalidateManagerCache(void) const;
  void invalidateManagerInstanceCache(void) const;

 private:
  const ICollisionContainer *m_CollisionContainer;
  const CollisionObject *m_parent;
  const IFCLCollisionObjectGroup *m_parent_interface;
  mutable std::unique_ptr<fcl::BroadPhaseCollisionManager<FCL_PRECISION>>
      m_group_manager;
  mutable BroadPhaseManagerFactoryConstPtr m_broadphase_factory;
  mutable bool manager_cached;
  mutable bool manager_instance_cached;
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision

#endif /* COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_ */
