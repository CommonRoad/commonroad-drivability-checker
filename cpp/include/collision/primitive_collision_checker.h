#ifndef PRIMITIVE_COLLISION_CHECKER_H_
#define PRIMITIVE_COLLISION_CHECKER_H_

#include <sstream>
#include <vector>
#include "collision/collision_object.h"
#include "collision/narrowphase/rectangle_aabb.h"

#include "i_collision_checker.h"

namespace collision {

class PrimitiveCollisionChecker;
typedef std::shared_ptr<PrimitiveCollisionChecker> PrimitiveCollisionCheckerPtr;
typedef std::shared_ptr<const PrimitiveCollisionChecker>
    PrimitiveCollisionCheckerConstPtr;

/**
 * \brief PrimitiveCollisionChecker can group any kinds of CollisionObjects
 *
 * No accelerator structures are used to filter the number of collision checks
 *
 */
class PrimitiveCollisionChecker : public ICollisionChecker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void addCollisionObject(CollisionObjectConstPtr co);
  bool collide(CollisionObjectConstPtr co) const;
  bool collide(CollisionObjectConstPtr co,
               CollisionObjectConstPtr &obstacle) const;
  bool collide(CollisionObjectConstPtr co,
               std::vector<CollisionObjectConstPtr> &obstacles) const;
  PrimitiveCollisionCheckerPtr windowQuery(const RectangleAABB &aabb) const;
  PrimitiveCollisionCheckerPtr timeSlice(int time_idx) const;
  void print(std::ostringstream &stream) const;
  int numberOfObstacles() const;
  std::vector<CollisionObjectConstPtr> getObstacles() const;

 private:
  std::vector<CollisionObjectConstPtr> collision_objects_;
};

}  // namespace collision

#endif
