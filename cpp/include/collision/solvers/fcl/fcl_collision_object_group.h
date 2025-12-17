#ifndef COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_
#define COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_

#include <fcl/narrowphase/collision_object.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <Eigen/Dense>
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

  virtual ~FCLCollisionObjectGroup(void) {}

  int getManager_fcl(fcl::BroadPhaseCollisionManager<FCL_PRECISION> *&)
      const;

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
  mutable bool manager_cached = false;
  mutable bool manager_instance_cached = false;
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision

#endif /* COLLISION_FCL_FCL_COLLISION_OBJECT_GROUP_H_ */
