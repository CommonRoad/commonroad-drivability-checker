#ifndef TESTS_COLLISION_BROADPHASE_TEST_H_
#define TESTS_COLLISION_BROADPHASE_TEST_H_

namespace collision {
class CollisionChecker;
class ShapeGroup;
}  // namespace collision

#include "collision/collision_object.h"

#include <stdlib.h>
#include <vector>

namespace collision {
namespace test {

class CollisionCheckerTest {
 public:
  static bool run_test_collide(CollisionObjectConstPtr co,
                        const collision::CollisionChecker *cc);
  static bool run_test_collide_obstacle(CollisionObjectConstPtr co,
                                 const collision::CollisionChecker *cc);
  static bool run_test_collide_obstacles(CollisionObjectConstPtr co,
                                  const collision::CollisionChecker *cc);

 private:

  static bool test_collide(CollisionObjectConstPtr co,
                    const collision::CollisionChecker *cc);
  static bool test_collide_obstacle(CollisionObjectConstPtr co,
                             const collision::CollisionChecker *cc);
  static bool test_collide_obstacles(
      CollisionObjectConstPtr co, const collision::CollisionChecker *cc,
      std::vector<CollisionObjectConstPtr> &missed_obstacles,
      std::vector<CollisionObjectConstPtr> &missed_obstacles_primit);

  static bool cmp(const CollisionObjectConstPtr &a,
                  const CollisionObjectConstPtr &b);

};

class ShapeGroupTest {
 public:
  static bool run_test_collide(CollisionObjectConstPtr co, const ShapeGroup *sg);
  static bool run_test_collide(const ShapeGroup *sg1, const ShapeGroup *sg2);
  static bool test_overlap_map(
      const ShapeGroup *sg1, const ShapeGroup *sg2,
      std::vector<std::pair<int, int>> *missed_col_naive_out = 0);
 private:
  static bool cmp(const std::pair<int, int> &pair1,
                  const std::pair<int, int> &pair2);
  static bool test_collide(CollisionObjectConstPtr co, const ShapeGroup *sg);
  static bool test_collide(const ShapeGroup *sg1, const ShapeGroup *sg2);

};
}  // namespace test
}  // namespace collision

#endif /* TESTS_COLLISION_BROADPHASE_TEST_H_ */
