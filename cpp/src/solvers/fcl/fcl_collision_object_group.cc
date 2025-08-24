#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/fcl_helpers.h"

namespace collision {
namespace solvers {
namespace solverFCL {

FCLCollisionObjectGroup::FCLCollisionObjectGroup(
    const ICollisionContainer *cont, const CollisionObject *parent,
    const IFCLCollisionObjectGroup *parent_interface)
    : m_CollisionContainer(cont),
      m_parent(parent),
      m_parent_interface(parent_interface) {
}

int FCLCollisionObjectGroup::getManager_fcl(
    fcl::BroadPhaseCollisionManager<FCL_PRECISION> *&mngr) const {
  if (!manager_cached) {
    if (!manager_instance_cached) {
      m_group_manager =
          std::unique_ptr<fcl::BroadPhaseCollisionManager<FCL_PRECISION>>(
        		  new fcl::DynamicAABBTreeCollisionManager<FCL_PRECISION>());
      manager_instance_cached = true;
    }
    m_group_manager->clear();
    std::vector<fcl::CollisionObject<FCL_PRECISION> *> objects;
    m_parent_interface->getCollisionObjects(objects);
    m_group_manager->registerObjects(objects);
    m_group_manager->setup();
    manager_cached = true;
  }
  mngr = m_group_manager.get();
  return 0;
}
void FCLCollisionObjectGroup::invalidateManagerCache(void) const {
  manager_cached = false;
}
void FCLCollisionObjectGroup::invalidateManagerInstanceCache(void) const {
  manager_instance_cached = false;
}

}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
