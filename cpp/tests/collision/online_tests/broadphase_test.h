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

int in_diagnose_struct(const CollisionObject *obj);
int log_parentmap_failure(CollisionObjectConstPtr co);

int log_broadphase_failure(CollisionObjectConstPtr co, const ShapeGroup *sg);
int log_broadphase_failure(CollisionObjectConstPtr co,
                           const CollisionChecker *cc);
int log_broadphase_failure(const ShapeGroup *sg1, const ShapeGroup *sg2);

bool test_tolerance_check(CollisionObjectConstPtr co,
                          const CollisionChecker *cc);
int serialize_broadphase_failure_test(const CollisionChecker *cc,
                                      const CollisionObject *obj);
int serialize_broadphase_failure_test(const CollisionObject *obj1,
                                      const CollisionObject *obj2);
int serialize_narrowphase_failure(ShapeConstPtr co, const ShapeGroup *sg,
                                  const std::vector<int> &failure_export);
int serialize_narrowphase_failure(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    const std::vector<std::pair<int, int>> &failure_pair_export);
int serialize_narrowphase_failure(CollisionObjectConstPtr co,
                                  const CollisionChecker *cc,
                                  const std::vector<int> &failure_export);
int serialize_narrowphase_failure_helper_obj(CollisionObjectConstPtr obj1,
                                             CollisionObjectConstPtr obj2);
int serialize_narrowphase_failure_helper(ShapeConstPtr obj1,
                                         ShapeConstPtr obj2);

int log_narrowphase_failure(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    const std::vector<std::pair<int, int>> &failure_pair_export);
int log_narrowphase_failure(CollisionObjectConstPtr co, const ShapeGroup *sg,
                            const std::vector<int> &failure_export);
std::string get_free_file(std::string prefix, int max_files,
                          std::string directory);

class CollisionCheckerTest {
 public:
  bool run_test_collide(CollisionObjectConstPtr co,
                        const collision::CollisionChecker *cc);
  bool run_test_collide_obstacle(CollisionObjectConstPtr co,
                                 const collision::CollisionChecker *cc);
  bool run_test_collide_obstacles(CollisionObjectConstPtr co,
                                  const collision::CollisionChecker *cc);

 private:
  // TODO: test collision_time
  // CollisionChecker currently always uses FCL for elementary object-object
  // checks

  bool test_collide(CollisionObjectConstPtr co,
                    const collision::CollisionChecker *cc);
  bool test_collide_obstacle(CollisionObjectConstPtr co,
                             const collision::CollisionChecker *cc);
  bool test_collide_obstacles(
      CollisionObjectConstPtr co, const collision::CollisionChecker *cc,
      std::vector<CollisionObjectConstPtr> &missed_obstacles,
      std::vector<CollisionObjectConstPtr> &missed_obstacles_primit);
  static bool cmp(const CollisionObjectConstPtr &a,
                  const CollisionObjectConstPtr &b);

  bool test_narrowphase(CollisionObjectConstPtr co, const CollisionChecker *cc,
                        std::vector<int> *failure_export);
};

class ShapeGroupTest {
 public:
  bool run_test_collide(CollisionObjectConstPtr co, const ShapeGroup *sg);
  bool run_test_collide(const ShapeGroup *sg1, const ShapeGroup *sg2);
  bool test_overlap_map(
      const ShapeGroup *sg1, const ShapeGroup *sg2,
      std::vector<std::pair<int, int>> *missed_col_naive_out = 0);

  bool run_test_narrowphase(CollisionObjectConstPtr co, const ShapeGroup *sg);
  bool run_test_narrowphase(const ShapeGroup *sg1, const ShapeGroup *sg2);

 private:
  static bool cmp(const std::pair<int, int> &pair1,
                  const std::pair<int, int> &pair2);
  bool test_collide(CollisionObjectConstPtr co, const ShapeGroup *sg);
  bool test_collide(const ShapeGroup *sg1, const ShapeGroup *sg2);
  bool test_narrowphase(CollisionObjectConstPtr co, const ShapeGroup *sg,
                        std::vector<int> *failure_export = 0);
  bool test_narrowphase(
      const ShapeGroup *sg1, const ShapeGroup *sg2,
      std::vector<std::pair<int, int>> *failure_pair_export = 0);
};
}  // namespace test
}  // namespace collision

#endif /* TESTS_COLLISION_BROADPHASE_TEST_H_ */
