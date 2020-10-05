#include "collision/solvers/fcl/fcl_collision_queries.h"

namespace collision

{
namespace solvers {
namespace solverFCL {
CollisionObjectConstPtr errObj;

CollisionObject *getParentPointerFromFclObj(
    fcl::CollisionObject<FCL_PRECISION> *fcl_obj) {
  CollisionObjectConstPtr ret_obj;

  if (fcl_obj) {
    void *user_data = fcl_obj->getUserData();
    if (user_data) {
      return static_cast<CollisionObject *>(user_data);
    }
  }

  throw;
  return 0;
}
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
