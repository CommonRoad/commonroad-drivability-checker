#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/fcl_distance_queries.h"
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
  invalidateManagerCache();
  invalidateManagerInstanceCache();
  m_broadphase_factory = getDefaultBroadphaseFactory();
}
FCLCollisionObjectGroup::FCLCollisionObjectGroup(
    const FCLCollisionObjectGroup &copy) {
  m_CollisionContainer = copy.m_CollisionContainer;
  m_parent = copy.m_parent;
  m_parent_interface = copy.m_parent_interface;

  invalidateManagerCache();
  invalidateManagerInstanceCache();
  m_broadphase_factory = copy.m_broadphase_factory;
}

int FCLCollisionObjectGroup::replaceBroadphaseFactory(
    BroadPhaseManagerFactoryConstPtr broadphase_factory) {
  m_broadphase_factory = broadphase_factory;
  invalidateManagerInstanceCache();
  invalidateManagerCache();  // recreate BV hierarchy when broadphase manager is
                             // replaced
  return 0;
}

int FCLCollisionObjectGroup::calculateDistance(const CollisionObject &obj2,
                                               FCL_PRECISION &distance,
                                               FCL_PRECISION tolerance) const {
  SolverEntity_FCL *obj2_entity;
  FCL_COLLISION_ENTITY_TYPE obj2_entity_type =
      get_object_fcl_entity_type(&obj2, obj2_entity);

  if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    int result = fcl_primitive_queries::FCL_CalculateDistance(
        *this, *(static_cast<const FCLCollisionObject *>(obj2_entity)),
        distance, tolerance);
    return result;
  } else if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
    int result = fcl_primitive_queries::FCL_CalculateDistance(
        *this, *(static_cast<const FCLCollisionObjectGroup *>(obj2_entity)),
        distance, tolerance);
    return result;
  } else
    return -1;
}

int FCLCollisionObjectGroup::calculateDistanceNegTolerance(
    const CollisionObject &obj2, double &distance,
    FCL_TOLERANCE_CHECK_TYPE check_type, FCL_PRECISION tolerance) const {
  SolverEntity_FCL *obj2_entity;
  FCL_COLLISION_ENTITY_TYPE obj2_entity_type =
      get_object_fcl_entity_type(&obj2, obj2_entity);
  if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return fcl_primitive_queries::FCL_DistanceTolerance(
        *this, *(static_cast<const FCLCollisionObject *>(obj2_entity)),
        distance, check_type, tolerance);

  } else if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
    return fcl_primitive_queries::FCL_DistanceTolerance(
        *this, *(static_cast<const FCLCollisionObjectGroup *>(obj2_entity)),
        distance, check_type, tolerance);
  } else
    return -1;
}

/*
int FCLCollisionObjectGroup::ToleranceCheck(CollisionObjectConstPtr obj2, bool&
result, double& distance, FCL_TOLERANCE_CHECK_TYPE check_type, FCL_PRECISION
tolerance) const
{
SolverEntity_FCL* obj2_entity;
FCL_COLLISION_ENTITY_TYPE obj2_entity_type = get_object_fcl_entity_type(obj2,
obj2_entity);

if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECT)
{
int tolerance_result = fcl_primitive_queries::FCL_DistanceTolerance(*this,
*(static_cast<const FCLCollisionObject*>(obj2_entity)), distance,check_type,
tolerance); if (tolerance_result >= 0)
{
if (distance > -1 * (abs(tolerance)))
result = true;
else
result = false;
}
return tolerance_result;
}
else if (obj2_entity_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP)
{
int tolerance_result = fcl_primitive_queries::FCL_DistanceTolerance(*this,
*(static_cast<const FCLCollisionObjectGroup*>(obj2_entity)), distance,
check_type, tolerance); if (distance > -1 * (abs(tolerance))) result = true;
else
result = false;
return tolerance_result;
}
else
return -1;
}
*/

int FCLCollisionObjectGroup::getManager_fcl(
    fcl::BroadPhaseCollisionManager<FCL_PRECISION> *&mngr) const {
  // todo: add a shared pointer for better concurrency?
  if (!manager_cached) {
    if (!manager_instance_cached) {
      m_group_manager =
          std::unique_ptr<fcl::BroadPhaseCollisionManager<FCL_PRECISION>>(
              m_broadphase_factory->instantiateBroadphaseManager());
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
