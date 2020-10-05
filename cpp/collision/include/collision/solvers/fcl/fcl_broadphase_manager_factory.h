#ifndef COLLISION_FCL_FCL_BROADPHASE_MANAGER_FACTORY_H_
#define COLLISION_FCL_FCL_BROADPHASE_MANAGER_FACTORY_H_

#include <fcl/broadphase/broadphase_collision_manager.h>
#include "collision/solvers/fcl/fcl_decl.h"

namespace collision

{
namespace solvers {
namespace solverFCL {
class BroadPhaseManagerFactory {
 public:
  virtual fcl::BroadPhaseCollisionManager<FCL_PRECISION>*
  instantiateBroadphaseManager(void) const = 0;
  virtual ~BroadPhaseManagerFactory(){};
};

class DynamicAABBTreeManagerFactory : public BroadPhaseManagerFactory {
 public:
  fcl::BroadPhaseCollisionManager<FCL_PRECISION>* instantiateBroadphaseManager(
      void) const;
  virtual ~DynamicAABBTreeManagerFactory(){};
};

typedef std::shared_ptr<BroadPhaseManagerFactory> BroadPhaseManagerFactoryPtr;
typedef std::shared_ptr<const BroadPhaseManagerFactory>
    BroadPhaseManagerFactoryConstPtr;

BroadPhaseManagerFactoryConstPtr getDefaultBroadphaseFactory(void);
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision

#endif /* COLLISION_FCL_FCL_BROADPHASE_MANAGER_FACTORY_H_ */
