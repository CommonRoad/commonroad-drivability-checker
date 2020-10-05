#ifndef CPP_COLLISION_ICOLLISIONCHECKER_H_
#define CPP_COLLISION_ICOLLISIONCHECKER_H_

#include "collision/collision_object.h"

namespace collision {

class ICollisionChecker;

/**
 * \brief External interface to a CollisionChecker (with or without an
 acceleration structure)

 */
class ICollisionChecker
    : public std::enable_shared_from_this<ICollisionChecker> {
 public:
  virtual void addCollisionObject(CollisionObjectConstPtr co) = 0;
  virtual int numberOfObstacles() const = 0;
  virtual std::vector<CollisionObjectConstPtr> getObstacles() const = 0;
  virtual ~ICollisionChecker() {}
};

}  // namespace collision

#endif /* CPP_COLLISION_ICOLLISIONCHECKER_H_ */
