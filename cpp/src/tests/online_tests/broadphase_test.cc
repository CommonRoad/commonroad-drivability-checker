#include "collision/application_settings.h"
#if ENABLE_COLLISION_TESTS
#include "collision/collision_checker.h"
#include "collision/collision_object.h"
#include "collision/shape_group.h"

#include "collision/narrowphase/rectangle_aabb.h"

#include "collision/solvers/fcl/fcl_collision_queries.h"

#include "collision/serialize/public/serialize_public.h"

#include "collision/solvers/distance_queries.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "collision/tests/broadphase_test.h"
#include "collision/tests/test_common.h"
#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace test {
// tests collision checker broadphase algorithms
// all narrowphase checks are assumed to be done with FCL solver

bool CollisionCheckerTest::test_collide(CollisionObjectConstPtr co,
                                        const CollisionChecker *cc) {
  // int col_time;
  bool res_collision_checker = cc->collide(co, 0, false);

  std::vector<collision::CollisionObjectConstPtr> obstacles =
      cc->getObstacles();

  bool res_bruteforce = false;

  for (auto &obstacle : obstacles) {
    if (obstacle->collide(*(co.get()), CollisionRequest(COL_FCL))) {
      res_bruteforce = true;
      break;
    }
  }
  if (res_collision_checker != res_bruteforce) {
    return false;
  } else {
    return true;
  }
}

bool CollisionCheckerTest::test_collide_obstacle(CollisionObjectConstPtr co,
                                                 const CollisionChecker *cc) {
  collision::RectangleAABBConstPtr rect1(
      new collision::RectangleAABB(0, 0, Eigen::Vector2d(0, 0)));

  CollisionObjectConstPtr obst_cc = rect1;
  bool res_cc = cc->collide(co, obst_cc, false, false, false);

  CollisionObjectConstPtr obst_primit = rect1;
  bool res_primitive = false;
  for (auto &c : cc->collision_objects_) {
    if (c->collide(*co, CollisionRequest(COL_FCL))) {
      obst_primit = c;
      res_primitive = true;
      break;
    }
  }

  if (res_cc != res_primitive) {
    return false;
  } else {
    return true;
  }
}

bool ShapeGroupTest::cmp(const std::pair<int, int> &pair1,
                         const std::pair<int, int> &pair2) {
  if (pair1.first != pair2.first) {
    return pair1.first < pair2.first;
  }
  return pair1.second > pair2.second;
  return false;
}

bool CollisionCheckerTest::cmp(const CollisionObjectConstPtr &a,
                               const CollisionObjectConstPtr &b) {
  return (a.get() < b.get());
}

bool CollisionCheckerTest::test_narrowphase(CollisionObjectConstPtr co,
                                            const CollisionChecker *cc,
                                            std::vector<int> *failure_export)

{
  if (!co.get()) return false;
  // const FCLCollisionObject* co_fcl = get_fcl_object_ptr(co.get());
  // if (!co_fcl) return false;

  std::vector<collision::CollisionObjectConstPtr> children = cc->getObstacles();
  int counter = 0;
  bool res = true;
  for (auto &el : children) {
    bool res_fcl = el->collide(*(co.get()), CollisionRequest(COL_FCL));
    // bool res_prim = el->collide(*(co.get()),
    // CollisionRequest(COL_PRIMITIVE));
    bool res_def = el->collide(*(co.get()), CollisionRequest(COL_DEFAULT));
    // bool res_local =  (res_fcl==res_prim && res_prim==res_def &&
    // res_def==res_fcl) ;
    bool res_local = (res_def == res_fcl);
    res = res_local && res;
    if (!res_local) {
      if (failure_export) failure_export->push_back(counter);
    }

    counter++;
  }
  return res;
}

bool CollisionCheckerTest::test_collide_obstacles(
    CollisionObjectConstPtr co, const CollisionChecker *cc,
    std::vector<CollisionObjectConstPtr> &missed_obstacles,
    std::vector<CollisionObjectConstPtr> &missed_obstacles_primit) {
#if ENABLE_SERIALIZER_TESTS
  if (serialize_broadphase_failure_test(cc, co.get())) {
    std::cout << "serialization CC_OBJ failed";
  }
#endif
  std::vector<int> failure_export;
#if ENABLE_COLLISION_TESTS_NARROWPHASE
  if (!test_narrowphase(co, cc, &failure_export)) {
    std::vector<int> failure_export2;
    test_narrowphase(co, cc, &failure_export2);
    test::serialize_narrowphase_failure(co, cc, failure_export);
  }
#endif
label1:
  std::vector<CollisionObjectConstPtr> obstacles_cc;
  // fcl_cc.collideHelper(co,0,&obstacles, -1, false, false);
  bool res_cc = cc->collide(co, obstacles_cc, false, false, false);
  /*if (obstacles_cc.size()) // simulates a broadphase failure
  {
          for (auto iter= obstacles_cc.begin(); iter<obstacles_cc.end(); iter++)
          {
                  if ((*iter)->getCollisionObjectClass() ==
  collision::OBJ_CLASS_SHAPE)
                  {
                          obstacles_cc.erase(iter);
                          break;
                  }
          }

  }
  */
  std::vector<CollisionObjectConstPtr> obstacles_primit;
  bool res_primitive = false;
  for (auto &c : cc->collision_objects_) {
    if (c->collide(*co, CollisionRequest(COL_FCL))) {
      obstacles_primit.push_back(c);
      res_primitive = true;
    }
  }

  std::unordered_map<const CollisionObject *, bool> obstacles_cc_map;
  for (auto &c : obstacles_cc) {
    obstacles_cc_map.emplace(c.get(), true);
  }

  std::unordered_map<const CollisionObject *, bool> obstacles_primit_map;
  for (auto &c : obstacles_primit) {
    obstacles_primit_map.emplace(c.get(), true);
  }

  if ((res_cc == false) && (res_primitive == false)) {
    if ((obstacles_cc.size() == 0) && (obstacles_primit.size() == 0)) {
      return true;
    }
  } else {
    for (auto &c : obstacles_primit) {
      if (obstacles_cc_map.find(c.get()) == obstacles_cc_map.end()) {
        missed_obstacles.push_back(c);
      }
    }
    for (auto &c : obstacles_cc) {
      if (obstacles_primit_map.find(c.get()) == obstacles_primit_map.end()) {
        missed_obstacles_primit.push_back(c);
      }
    }

    if (res_cc == res_primitive) {
      if ((obstacles_cc.size() == obstacles_primit.size())) {
        std::sort(obstacles_cc.begin(), obstacles_cc.end(), cmp);
        std::sort(obstacles_primit.begin(), obstacles_primit.end(), cmp);

        if (obstacles_cc == obstacles_primit) {
          return true;
        } else
          return false;

      } else {
        std::sort(obstacles_cc.begin(), obstacles_cc.end(), cmp);
        std::sort(obstacles_primit.begin(), obstacles_primit.end(), cmp);
      }
    }
  }

  return false;
}

// tests ShapeGroup broadphase algorithms
// all narowphase checks are supposed to be done with FCL solver

bool ShapeGroupTest::test_collide(CollisionObjectConstPtr co,
                                  const ShapeGroup *sg) {
  bool res_sg = false;
  const FCLCollisionObjectGroup *sg_fcl = get_fcl_object_group_ptr(sg);
  const FCLCollisionObject *co_fcl = get_fcl_object_ptr(co.get());
  if (!sg_fcl || !co_fcl) return false;

  res_sg = fcl_primitive_queries::fcl_generic_collisionDetection(
      *sg_fcl, *co_fcl, false);

  std::vector<collision::ShapeConstPtr> obstacles = sg->shapes_;

  bool res_bruteforce = false;

  for (auto &obstacle : obstacles) {
    if (obstacle->collide(*(co.get()), CollisionRequest(COL_FCL))) {
      res_bruteforce = true;
      break;
    }
  }
  if (res_bruteforce != res_sg) {
    int cont = 1;
  }
  return res_bruteforce == res_sg;
}
bool ShapeGroupTest::test_collide(const ShapeGroup *sg1,
                                  const ShapeGroup *sg2) {
  bool res_sg = false;
  const FCLCollisionObjectGroup *sg1_fcl = get_fcl_object_group_ptr(sg1);
  const FCLCollisionObjectGroup *sg2_fcl = get_fcl_object_group_ptr(sg2);
  if (!sg1_fcl || !sg2_fcl) return false;

  res_sg = fcl_primitive_queries::fcl_generic_collisionDetection(
      *sg1_fcl, *sg2_fcl, false);

  std::vector<collision::ShapeConstPtr> obstacles1 = sg1->unpack();
  std::vector<collision::ShapeConstPtr> obstacles2 = sg2->unpack();

  bool res_bruteforce = false;

  std::vector<std::pair<int, int>> overlap_res = sg1->overlap(*sg2);
  bool res2 = false;
  for (auto &res1 : overlap_res) {
    if (obstacles1[res1.first]->collide(*obstacles2[res1.second])) {
      res2 = true;
    }
  }
  for (auto obstacle1 : obstacles1) {
    for (auto obstacle2 : obstacles2) {
      if (obstacle1->collide(*obstacle2, CollisionRequest(COL_FCL))) {
        res_bruteforce = true;
        break;
      }
    }
  }
  return res_bruteforce == res_sg;
}

bool ShapeGroupTest::test_narrowphase(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    std::vector<std::pair<int, int>> *failure_pair_export)

{
  std::vector<collision::ShapeConstPtr> obstacles1 = sg1->shapes_;
  int counter1 = 0;
  std::vector<int> failure_export;
  bool res = true;
  for (auto &obstacle : obstacles1) {
    failure_export.clear();
    res = test_narrowphase(obstacle, sg2, &failure_export) && res;
    for (auto el : failure_export) {
      if (failure_pair_export) failure_pair_export->emplace_back(counter1, el);
    }
    counter1++;
  }
  return res;
}

bool ShapeGroupTest::test_narrowphase(CollisionObjectConstPtr co,
                                      const ShapeGroup *sg,
                                      std::vector<int> *failure_export)

{
  const FCLCollisionObject *co_fcl = get_fcl_object_ptr(co.get());
  if (!co_fcl) return false;

  std::vector<collision::ShapeConstPtr> shapes = sg->shapes_;
  int counter = 0;
  bool res = true;
  for (auto &shape : shapes) {
    if (shape->type() == TYPE_POLYGON) {
      collision::ShapeGroup triangles_sg;
      const collision::Polygon *shape_polyg =
          static_cast<const collision::Polygon *>(shape.get());
      std::vector<TriangleConstPtr> triangles = shape_polyg->getTriangleMesh();
      for (auto &tri : triangles) {
        triangles_sg.addToGroup(tri);
      }
      bool res_local = ShapeGroupTest::test_narrowphase(co, &triangles_sg, 0);
      res = res_local && res;
      if (!res_local) {
        if (failure_export) failure_export->push_back(counter);
      }

    } else {
      // bool res_local = shape->collide(*(co.get()), CollisionRequest(COL_FCL))
      // == shape->collide(*(co.get()), CollisionRequest(COL_PRIMITIVE));
      bool res_local =
          shape->collide(*(co.get()), CollisionRequest(COL_FCL)) ==
          shape->collide(*(co.get()), CollisionRequest(COL_DEFAULT));
      res = res_local && res;
      if (!res_local) {
        if (failure_export) failure_export->push_back(counter);
      }
    }
    counter++;
  }
  return res;
}

bool CollisionCheckerTest::run_test_collide(CollisionObjectConstPtr co,
                                            const CollisionChecker *cc) {
  bool res = test_collide(co, cc);
  if (!res) {
    log_broadphase_failure(co, cc);
  }
  return res;
}

bool CollisionCheckerTest::run_test_collide_obstacle(
    CollisionObjectConstPtr co, const CollisionChecker *cc) {
  bool res = test_collide_obstacle(co, cc);
  if (!res) {
    log_broadphase_failure(co, cc);
  }
  return res;
}

bool CollisionCheckerTest::run_test_collide_obstacles(
    CollisionObjectConstPtr co, const CollisionChecker *cc) {
  if (0 && serialize_broadphase_failure_test(cc, co.get())) {
    std::cout << "serialization CC obj failed";
  }
  std::vector<CollisionObjectConstPtr> missed_obstacles;
  std::vector<CollisionObjectConstPtr> missed_obstacles_primit;
  bool res =
      test_collide_obstacles(co, cc, missed_obstacles, missed_obstacles_primit);
  if (!res) {
    // log_broadphase_failure(co, cc);
    CollisionChecker cc2;
    for (auto &obst : missed_obstacles) {
      // bool naive_collide = obst->collide(*co,
      // CollisionRequest(COL_PRIMITIVE));
      bool naive_collide = obst->collide(*co, CollisionRequest(COL_DEFAULT));
      bool non_naive_collide = obst->collide(*co, CollisionRequest(COL_FCL));
      cc2.addCollisionObject(obst);
    }
    for (auto &obst : missed_obstacles_primit) {
      // bool naive_collide = obst->collide(*co,
      // CollisionRequest(COL_PRIMITIVE));
      bool naive_collide = obst->collide(*co, CollisionRequest(COL_DEFAULT));
      bool non_naive_collide = obst->collide(*co, CollisionRequest(COL_FCL));
      std::cout << "narrowphase internal failure";
    }
    bool tolerance_result = test_tolerance_check(co, &cc2);
    std::vector<CollisionObjectConstPtr> new_obstacles;
    if (tolerance_result) {
      test_tolerance_check(co, &cc2);
      cc->collide(co, new_obstacles, false, false, false);
      cc->collide(co);
      log_broadphase_failure(co, cc);
    }
    for (auto &obst : missed_obstacles) {
      for (auto &obst2 : new_obstacles) {
        if (obst == obst2) {
          std::cout << "found";
        }
      }
    }
  }
  return res;
}

int ToleranceCheck(CollisionObjectConstPtr obj1, CollisionObjectConstPtr obj2,
                   bool &tolerance_result, double &distance,
                   bool narrowphase = false) {
  DistanceRequest req;
  DistanceResult res;
  req.dist_request_type = DistanceRequestType::DRT_TOLERANCE_NEG;
#if TOLERANCE_BBONLY
  req.dist_node_type = DistanceNodeType::DNT_AABB;
#else
  req.dist_node_type = DistanceNodeType::DNT_NARROWPHASE;
#endif
  if (narrowphase) {
    req.dist_node_type = DistanceNodeType::DNT_NARROWPHASE;
  }
  if (collision::distance(*obj1, *obj2, res, req) >= 0) {
    tolerance_result = res.getTolerancePassed();
    return 0;
  } else {
    tolerance_result = false;
    return -1;
  }
}

bool ShapeGroupTest::test_overlap_map(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    std::vector<std::pair<int, int>> *missed_col_naive_out) {
  std::vector<std::pair<int, int>> res = sg1->overlap(*sg2);
  std::vector<collision::ShapeConstPtr> obstacles1 = sg1->unpack();
  std::vector<collision::ShapeConstPtr> obstacles2 = sg2->unpack();

  std::vector<std::pair<int, int>> overlap_res = sg1->overlap(*sg2);
  std::vector<std::pair<int, int>> overlap_res2;
  bool res2 = false;
  for (int cc1 = 0; cc1 < obstacles1.size(); cc1++) {
    for (int cc2 = 0; cc2 < obstacles2.size(); cc2++) {
      if (obstacles1[cc1]->collide(*obstacles2[cc2],
                                   CollisionRequest(COL_FCL))) {
        overlap_res2.emplace_back(cc1, cc2);
      }
    }
  }
  /*
  if (overlap_res.size()) // simulates a broadphase failure
  {
          for (auto iter= overlap_res.begin(); iter<overlap_res.end(); iter++)
          {

                  overlap_res.erase(iter);
                  break;
          }

  }
  */

  // test function returning ordered list of pairs

  std::sort(overlap_res.begin(), overlap_res.end(), cmp);
  std::sort(overlap_res2.begin(), overlap_res2.end(), cmp);
  std::vector<std::pair<int, int>> missed_col;

  bool local_failure1 = !(overlap_res == overlap_res2);
  if (local_failure1) {
    local_failure1 = false;
    std::map<std::pair<int, int>, bool> overlap_res_hash;
    for (auto res1 : overlap_res) {
      overlap_res_hash.emplace(res1, true);
    }
    for (auto res2 : overlap_res2) {
      if (overlap_res_hash.find(res2) == overlap_res_hash.end()) {
        missed_col.push_back(res2);
      }
    }
    for (auto miss : missed_col) {
      bool tolerance_result = false;
      double distance;
      int tol_res =
          ToleranceCheck(obstacles1[miss.first], obstacles2[miss.second],
                         tolerance_result, distance);

      if (!tolerance_result) {
        local_failure1 = true;
        break;
      }
    }
  }

  if (overlap_res.size() + missed_col.size() >
      overlap_res2.size())  // if returned extra pairs
  {
    std::map<std::pair<int, int>, bool> overlap_res2_hash;
    for (auto res2 : overlap_res2) {
      overlap_res2_hash.emplace(res2, true);
    }
    std::vector<std::pair<int, int>> missed_col_naive;
    for (auto res : overlap_res) {
      if (overlap_res2_hash.find(res) == overlap_res2_hash.end()) {
        missed_col_naive.push_back(res);
      }
    }

    for (auto miss : missed_col_naive) {
      bool tolerance_result = false;
      bool tolerance_result2 = false;
      double distance;
      int tol_res =
          ToleranceCheck(obstacles1[miss.first], obstacles2[miss.second],
                         tolerance_result, distance);
      if (!tolerance_result) {
        local_failure1 = true;
        break;
      }
    }

    if (missed_col_naive_out) {
      *missed_col_naive_out = missed_col_naive;
    }
    // local_failure1 = true;
  }
  // test that after hashmapping no new collisions are listed

  if (local_failure1) return false;

  std::map<std::pair<int, int>, bool> overlap_res_hash;
  for (auto res2 : overlap_res) {
    overlap_res_hash.emplace(res2, true);
  }

  std::vector<std::set<int>> overlap_map_result = sg1->overlapMap(*sg2);
  bool true_failure = true;
  bool local_failure2 = false;
  for (int cc1 = 0; cc1 < overlap_map_result.size(); cc1++) {
    std::set<int> iter1 = overlap_map_result[cc1];
    for (auto cc2 : iter1) {
      if (overlap_res_hash.find(std::pair<int, int>(cc1, cc2)) ==
          overlap_res_hash.end()) {
        local_failure2 = true;
        break;
      }
    }
    if (local_failure2) break;
  }
  if (local_failure2) return false;

  // test that no collisions are missed after hashmapping
  bool local_failure3 = false;
  for (auto res1 : overlap_res) {
    if (overlap_map_result[res1.first].find(res1.second) ==
        overlap_map_result[res1.first].end()) {
      // overlap pair not found in the overlap_map result
      local_failure3 = true;
      break;
    }
  }
  if (local_failure3) return false;
  true_failure = local_failure1 || local_failure2 || local_failure3;

  return !true_failure;
}

bool ShapeGroupTest::run_test_collide(CollisionObjectConstPtr co,
                                      const ShapeGroup *sg) {
  bool res = test_collide(co, sg);
#if ENABLE_SERIALIZER_TESTS
  if (serialize_broadphase_failure_test(sg, co.get())) {
    std::cout << "serializer OBJ OBJ failed";
  }
#endif
  if (!res) {
    bool local_failure1 = false;
    for (auto el : sg->unpack()) {
      bool tolerance_result = false;
      double distance;
      int tol_res = ToleranceCheck(el, co, tolerance_result, distance);
      if (!tolerance_result) {
        local_failure1 = true;
        break;
      }
    }
    if (local_failure1) {
      res = test_collide(co, sg);

      log_broadphase_failure(co, sg);
    }
  }
  return res;
}
bool ShapeGroupTest::run_test_collide(const ShapeGroup *sg1,
                                      const ShapeGroup *sg2) {
  bool res = test_collide(sg1, sg2);
  // bool res2=true;
  bool res2 = test_overlap_map(sg1, sg2);
  test::BroadphaseFailureObjObj bf;
  bf.obj1 = sg1->shared_from_this();
  bf.obj2 = sg2->shared_from_this();
#if ENABLE_SERIALIZER_TESTS
  if (serialize_broadphase_failure_test(sg1, sg2)) {
    std::cout << "serializer OBJ OBJ failed";
  }
#endif
  // if(1)
  if (!res || !res2)
  // if (!res || !res2)
  {
    res = test_collide(sg1, sg2);
    res2 = test_overlap_map(sg1, sg2);
    log_broadphase_failure(sg1, sg2);
  }
  return res && res2;
}

bool ShapeGroupTest::run_test_narrowphase(const ShapeGroup *sg1,
                                          const ShapeGroup *sg2) {
  std::vector<std::pair<int, int>> failure_pair_export;
  bool res = test_narrowphase(sg1, sg2, &failure_pair_export);
  if (!res) {
    log_narrowphase_failure(sg1, sg2, failure_pair_export);
  }
  return res;
}

bool ShapeGroupTest::run_test_narrowphase(CollisionObjectConstPtr co,
                                          const ShapeGroup *sg) {
  std::vector<int> failure_export;
  bool res = test_narrowphase(co, sg, &failure_export);
  if (!res) {
    log_narrowphase_failure(co, sg, failure_export);
  }
  return res;
}
int serialize_narrowphase_failure_helper_obj(CollisionObjectConstPtr obj1,
                                             CollisionObjectConstPtr obj2) {
  int res = 0;
  // if ((obj1->getCollisionObjectClass() == collision::OBJ_CLASS_SHAPE) &&
  // (obj2->getCollisionObjectClass() == collision::OBJ_CLASS_SHAPE))
  //{
  //	return
  // serialize_narrowphase_failure_helper(std::static_pointer_cast<const
  // Shape>(obj1), std::static_pointer_cast<const Shape>(obj2));
  //}
  int type1 = obj1->getCollisionObjectType();
  int type2 = obj2->getCollisionObjectType();
  if (type1 > type2) {
    std::swap(type1, type2);
  }
  std::string descriptor = "narrowphase_failure_obj_obj";
  descriptor += std::to_string(type1) + std::string("_") +
                std::to_string(type2) + std::string(".dump");
  std::string filename =
      get_free_file(descriptor, 100, NARROWPHASE_DUMP_DIRECTORY);
  if (filename != std::string("")) {
    test::BroadphaseFailureObjObj failure_export;
    failure_export.obj1 = obj1;
    failure_export.obj2 = obj2;
    std::ofstream ofs(filename, std::ofstream::out);
    if (serialize::serialize(failure_export, ofs)) {
      res = -1;
    }
    ofs.close();
  }
  return res;
}

int serialize_narrowphase_failure_helper(ShapeConstPtr obj1,
                                         ShapeConstPtr obj2) {
  int res = 0;

  int type1 = obj1->type();
  int type2 = obj2->type();

  if (type1 > type2) {
    std::swap(type1, type2);
  }
  std::string descriptor = "narrowphase_failure_";
  descriptor += std::to_string(type1) + std::string("_") +
                std::to_string(type2) + std::string(".dump");
  std::string filename =
      get_free_file(descriptor, 100, NARROWPHASE_DUMP_DIRECTORY);
  if (filename != std::string("")) {
    test::BroadphaseFailureObjObj failure_export;
    failure_export.obj1 = obj1;
    failure_export.obj2 = obj2;
    std::ofstream ofs(filename, std::ofstream::out);
    if (serialize::serialize(failure_export, ofs)) {
      res = -1;
    }
    ofs.close();
  }

  bool tolerance_result;
  double distance;
  int tol_res = ToleranceCheck(obj1, obj2, tolerance_result, distance, true);
  if (!tolerance_result) {
    std::string descriptor = "narrowphase_failure_tolerance_checked";
    descriptor += std::to_string(type1) + std::string("_") +
                  std::to_string(type2) + std::string(".dump");
    std::string filename =
        get_free_file(descriptor, 100, NARROWPHASE_DUMP_DIRECTORY);
    if (filename != std::string("")) {
      test::BroadphaseFailureObjObj failure_export;
      failure_export.obj1 = obj1;
      failure_export.obj2 = obj2;
      std::ofstream ofs(filename, std::ofstream::out);
      if (serialize::serialize(failure_export, ofs)) {
        res = -1;
      }
      ofs.close();
    }
  }

  return res;
}

int serialize_narrowphase_failure(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    const std::vector<std::pair<int, int>> &failure_pair_export) {
  int res = 0;
  std::vector<collision::ShapeConstPtr> obstacles1 = sg1->unpack();
  std::vector<collision::ShapeConstPtr> obstacles2 = sg2->unpack();
  for (auto el : failure_pair_export) {
    ShapeConstPtr obj1 = obstacles1[el.first];
    ShapeConstPtr obj2 = obstacles2[el.second];
    if (serialize_narrowphase_failure_helper(obj1, obj2)) {
      res = -1;
    }
  }
  return res;
}
int serialize_narrowphase_failure(ShapeConstPtr co, const ShapeGroup *sg,
                                  const std::vector<int> &failure_export) {
  int res = 0;
  std::vector<collision::ShapeConstPtr> obstacles1 = sg->unpack();
  for (auto el : failure_export) {
    ShapeConstPtr obj1 = obstacles1[el];
    if (serialize_narrowphase_failure_helper(co, obj1)) {
      res = -1;
    }
  }
  return res;
}

int serialize_narrowphase_failure(CollisionObjectConstPtr co,
                                  const CollisionChecker *cc,
                                  const std::vector<int> &failure_export) {
  int res = 0;
  std::vector<collision::CollisionObjectConstPtr> obstacles1 =
      cc->getObstacles();
  for (auto el : failure_export) {
    if (serialize_narrowphase_failure_helper_obj(co, obstacles1[el])) {
      res = -1;
    }
  }
  return res;
}

int log_narrowphase_failure(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    const std::vector<std::pair<int, int>> &failure_pair_export) {
  serialize_narrowphase_failure(sg1, sg2, failure_pair_export);
  std::ostringstream outstrstream;
  outstrstream << "[Narrowphase failure]" << std::endl;
  // sg1->toString(outstrstream);
  // sg2->toString(outstrstream);
  outstrstream << "[/Narrowphase failure]" << std::endl;
  TestFailureLoggerNarrowphase logger;
  logger.log_failure(outstrstream.str());
  return 0;
}

int log_narrowphase_failure(CollisionObjectConstPtr co, const ShapeGroup *sg,
                            const std::vector<int> &failure_export) {
  if (co->getCollisionObjectClass() ==
      collision::CollisionObjectClass::OBJ_CLASS_SHAPE) {
    serialize_narrowphase_failure(std::static_pointer_cast<const Shape>(co), sg,
                                  failure_export);
    std::ostringstream outstrstream;
    outstrstream << "[Narrowphase failure]" << std::endl;
    // co->toString(outstrstream);
    // cc->toString(outstrstream);
    outstrstream << "[/Narrowphase failure]" << std::endl;
    TestFailureLoggerNarrowphase logger;
    logger.log_failure(outstrstream.str());
    return 0;
  }
  return 0;
}
int log_parentmap_failure(CollisionObjectConstPtr co) {
  std::ostringstream outstrstream;
  outstrstream << "[Parentmap failure]" << std::endl;
  co->toString(outstrstream);
  outstrstream << "[/Parentmap failure]" << std::endl;
  TestFailureLoggerParentMap logger;
  logger.log_failure(outstrstream.str());
  return 0;
}

int dirExists(const char *path) {
  struct stat info;

  if (stat(path, &info) != 0)
    return 0;
  else if (info.st_mode & S_IFDIR)
    return 1;
  else
    return 0;
}

std::string get_free_file(std::string prefix, int max_files,
                          std::string directory) {
  std::string full_name_test = directory;
  if (!dirExists(full_name_test.c_str())) {
    return std::string("");
  }
  for (int cc1 = 0; cc1 < max_files; cc1++) {
    full_name_test = directory + std::string("/") + prefix + std::string("_") +
                     std::to_string(cc1) + std::string(".dump");
    std::ifstream f(full_name_test.c_str());
    if (!f.good()) {
      return full_name_test;
    }
  }
  return std::string("");
}
int serialize_broadphase_failure_test(const CollisionObject *obj1,
                                      const CollisionObject *obj2) {
  test::BroadphaseFailureObjObj bf;
  bf.obj1 = obj1->shared_from_this();
  bf.obj2 = obj2->shared_from_this();
  std::stringstream strstr1;
  std::stringstream strstr2;
  std::string str1;
  std::string str2;
  serialize::serialize(bf, strstr1);
  str1 = strstr1.str();
  BroadphaseFailureObjObj bf_load;
  serialize::deserialize(bf_load, strstr1);
  if (bf_load.obj1.get() && bf_load.obj2.get()) {
    serialize::serialize(bf_load, strstr2);
    str2 = strstr2.str();
    bool passed = str1 == str2;
    if (passed) {
      return 0;
    } else
      return 1;
  } else {
    return -1;
  }
}
int serialize_broadphase_failure_test(const CollisionChecker *cc,
                                      const CollisionObject *obj) {
  test::BroadphaseFailureCCObj bf;
  bf.cc =
      std::static_pointer_cast<const CollisionChecker>(cc->shared_from_this());
  bf.obj = obj->shared_from_this();
  std::stringstream strstr1;
  std::stringstream strstr2;
  std::string str1;
  std::string str2;
  serialize::serialize(bf, strstr1);
  str1 = strstr1.str();
  BroadphaseFailureCCObj bf_load;
  serialize::deserialize(bf_load, strstr1);
  if (bf_load.cc.get() && bf_load.obj.get()) {
    serialize::serialize(bf_load, strstr2);
    str2 = strstr2.str();
    bool passed = str1 == str2;
    if (passed) {
      return 0;
    } else
      return 1;
  } else {
    return -1;
  }
}

int serialize_broadphase_failure(const CollisionChecker *cc,
                                 const CollisionObject *obj) {
  test::BroadphaseFailureCCObj bf;
  bf.cc =
      std::static_pointer_cast<const CollisionChecker>(cc->shared_from_this());
  bf.obj = obj->shared_from_this();
  std::string filename = get_free_file("broadphase_failure_cc_obj", 10000,
                                       BROADPHASE_DUMP_DIRECTORY);
  if (filename == "") return -1;
  std::ofstream ofs(filename, std::ofstream::out);
  serialize::serialize(bf, ofs);
  ofs.close();
  return 0;
}
int serialize_broadphase_failure(const CollisionObject *obj1,
                                 const CollisionObject *obj2) {
  test::BroadphaseFailureObjObj bf;
  bf.obj1 = obj1->shared_from_this();
  bf.obj2 = obj2->shared_from_this();
  std::string filename = get_free_file("broadphase_failure_obj_obj", 10000,
                                       BROADPHASE_DUMP_DIRECTORY);
  if (filename == "") return -1;
  std::ofstream ofs(filename, std::ofstream::out);
  serialize::serialize(bf, ofs);
  ofs.close();
  return 0;
}

int log_broadphase_failure(const ShapeGroup *sg1, const ShapeGroup *sg2) {
  serialize_broadphase_failure(sg1, sg2);
  std::ostringstream outstrstream;
  outstrstream << "[Broadphase failure]" << std::endl;
  sg1->toString(outstrstream);
  sg2->toString(outstrstream);
  outstrstream << "[/Broadphase failure]" << std::endl;
  TestFailureLoggerBroadphase logger;
  logger.log_failure(outstrstream.str());
  return 0;
}
bool test_tolerance_check(CollisionObjectConstPtr co,
                          const CollisionChecker *cc) {
  bool result = true;
  double distance = std::numeric_limits<FCL_PRECISION>::max();
  bool true_Failure = false;

  for (auto &obj : cc->getObstacles()) {
    if (obj->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE ||
        co->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
      bool tvgroup_tolerance_passed = true;
      int time_start_idx = 0, time_end_idx = 0;
      const TimeVariantCollisionObject *obj_tv = 0;
      const TimeVariantCollisionObject *co_tv = 0;
      if (co->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
        co_tv = static_cast<const TimeVariantCollisionObject *>(co.get());
      }
      if (obj->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
        obj_tv = static_cast<const TimeVariantCollisionObject *>(obj.get());
      }
      if (obj_tv && co_tv) {
        time_start_idx =
            std::min(obj_tv->time_start_idx(), co_tv->time_start_idx());
        time_end_idx = std::max(obj_tv->time_end_idx(), co_tv->time_end_idx());
      } else {
        if (obj_tv) {
          time_start_idx = obj_tv->time_start_idx();
          time_end_idx = obj_tv->time_end_idx();
        } else if (co_tv) {
          time_start_idx = co_tv->time_start_idx();
          time_end_idx = co_tv->time_end_idx();
        } else
          throw;
      }
      for (int time_idx = time_start_idx; time_idx < time_end_idx; time_idx++) {
        CollisionObjectConstPtr obj1(nullptr);
        if (obj_tv) {
          obj1 = obj_tv->getObstacleAtTime(time_idx);
        } else
          obj1 = obj;
        CollisionObjectConstPtr obj2(nullptr);
        if (co_tv)
          obj2 = co_tv->getObstacleAtTime(time_idx);
        else
          obj2 = co;
        if (!obj1.get() || !obj2.get()) {
          continue;
        }

        bool tol_res = false;
        double dist = 0;
        ToleranceCheck(obj1, obj2, tol_res, dist);
        if (!tol_res) {
          tvgroup_tolerance_passed = false;
          break;
        }
      }
      if (tvgroup_tolerance_passed) {
        result = true;
      } else {
        result = false;
      }
    } else {
      bool tol_res = false;
      double dist = 0;
      ToleranceCheck(obj, co, tol_res, dist);
      if (!tol_res) {
        result = false;
      }
    }
    if (!result) {
      true_Failure = true;
      break;
    }
  }
  return true_Failure;
}
int log_broadphase_failure(CollisionObjectConstPtr co,
                           const CollisionChecker *cc) {
  serialize_broadphase_failure(cc, co.get());
  std::ostringstream outstrstream;
  outstrstream << "[Broadphase failure]" << std::endl;
  co->toString(outstrstream);
  cc->toString(outstrstream);
  TestFailureLoggerBroadphase logger;

  logger.log_failure(outstrstream.str());
  return 0;
}
int log_broadphase_failure(CollisionObjectConstPtr co, const ShapeGroup *sg) {
  std::string outstr;
  std::ostringstream outstrstream;
  outstrstream << "[Broadphase failure]" << std::endl;
  co->toString(outstrstream);
  sg->toString(outstrstream);
  outstrstream << "[/Broadphase failure]" << std::endl;
  TestFailureLoggerBroadphase logger;
  logger.log_failure(outstrstream.str());
  return 0;
}

}  // namespace test
}  // namespace collision
#endif
