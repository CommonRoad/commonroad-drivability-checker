#ifndef COLLISION_COLLISION_CONTAINER_H_
#define COLLISION_COLLISION_CONTAINER_H_

#include "collision/collision_object.h"

namespace collision {
class ICollisionContainer {
 public:
  ICollisionContainer(){};
  virtual int queryContainedObjectIndexList(const CollisionObject *pObj,
                                            std::list<int> &retlist) const = 0;

  virtual ~ICollisionContainer(){

  };
};
}  // namespace collision

#endif /* COLLISION_COLLISION_CONTAINER_H_ */
