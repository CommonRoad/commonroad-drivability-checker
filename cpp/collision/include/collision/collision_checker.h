#ifndef COLLISION_CHECKER_H_
#define COLLISION_CHECKER_H_

#include <sstream>
#include <vector>

#include "collision/application.h"

#include "collision/primitive_collision_checker.h"

#include "collision/collision_object.h"
#include "collision/i_collision_checker.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/solvers/fcl/fcl_collision_checker.h"

#if ENABLE_SERIALIZER
#include "collision/i_collision_checker_export.h"
#endif

namespace collision {

class CollisionChecker;
typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;
typedef std::shared_ptr<const CollisionChecker> CollisionCheckerConstPtr;
}  // namespace collision

namespace collision {

namespace solvers {
namespace solverFCL {
class FCLCollisionChecker;
}
}  // namespace solvers

class CollisionChecker;
/*!
        \brief CollisionChecker can contain collision objects and their groups
   (such as ShapeGroup and TimeVariantCollisionObject)

        The benefit of grouping objects in the CollisionChecker is that a
   broadphase structure is built for efficient filtering of candidate objects
   for the collision

*/
class CollisionChecker : public ICollisionChecker {
#if ENABLE_COLLISION_TESTS == 1
  friend class collision::test::CollisionCheckerTest;
#endif

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionChecker() : fcl_cc_(*this){};

  CollisionCheckerPtr clone_shared(void) const;

  void addCollisionObject(
      CollisionObjectConstPtr co);  /// adds a collision object to the CC

#if ENABLE_SERIALIZER
  serialize::ICollisionCheckerExport *exportThis(void) const;

  int serialize(std::ostream &output_stream) const;
  static CollisionCheckerConstPtr deserialize(std::istream &output_stream);

#endif

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect,
                bool remove_overlaps = true);

  int collisionTime(CollisionObjectConstPtr co) const;
#if (ENABLE_COLLISION_TESTS)
  bool collide(CollisionObjectConstPtr co, int *collision_time = 0,
               bool enable_test = true) const;
  bool collide(CollisionObjectConstPtr co, CollisionObjectConstPtr &obstacle,
               bool ungroup_shape_groups = false,
               bool ungroup_TV_obstacles = false,
               bool enable_test = true) const;
  bool collide(CollisionObjectConstPtr co,
               std::vector<CollisionObjectConstPtr> &obstacles,
               bool ungroup_shape_groups = false,
               bool ungroup_TV_obstacles = false,
               bool enable_test = true) const;
#else
  bool collide(CollisionObjectConstPtr co, int *collision_time = 0) const;
  bool collide(CollisionObjectConstPtr co, CollisionObjectConstPtr &obstacle,
               bool ungroup_shape_groups = false,
               bool ungroup_TV_obstacles = false) const;
  bool collide(CollisionObjectConstPtr co,
               std::vector<CollisionObjectConstPtr> &obstacles,
               bool ungroup_shape_groups = false,
               bool ungroup_TV_obstacles = false) const;
#endif

  CollisionCheckerPtr windowQuery(const RectangleAABB &aabb) const;
  PrimitiveCollisionCheckerPtr windowQueryPrimitive(
      const RectangleAABB &aabb) const;
  CollisionCheckerPtr timeSlice(int time_idx) const;
  void print(std::ostringstream &stream) const;
  void toString(std::ostringstream &stream) const;
  int numberOfObstacles(void) const;
  std::vector<CollisionObjectConstPtr> getObstacles(void) const;

 private:
  std::vector<CollisionObjectConstPtr> collision_objects_;
  FCLCollisionChecker fcl_cc_;
};
}  // namespace collision
#endif
