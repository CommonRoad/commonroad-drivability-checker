#include "collision/application_settings.h"
#include "collision/collision_checker.h"
#include "collision/collision_object.h"
#include "collision/shape_group.h"

#include "collision/narrowphase/rectangle_aabb.h"

#include "collision/solvers/fcl/fcl_collision_queries.h"

#include <stdio.h>
#include <stdlib.h>

#include "collision/tests/broadphase_test2.h"

namespace collision {
namespace test {

bool test_tolerance_check(CollisionObjectConstPtr co,
                          const CollisionChecker *cc);
int aabb_tolerance_check(CollisionObjectConstPtr obj1, CollisionObjectConstPtr obj2, bool& tolerance_result, double& distance);

#define AABB_TOLERANCE_THRESHOLD 1e-10;

// tests collision checker broadphase algorithms
// all narrowphase checks are assumed to be done with FCL solver

bool CollisionCheckerTest::test_collide(CollisionObjectConstPtr co,
                                        const CollisionChecker *cc) {
  // int col_time;
  bool res_collision_checker = cc->collide(co, 0);

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
  bool res_cc = cc->collide(co, obst_cc, false, false);

  CollisionObjectConstPtr obst_primit = rect1;
  bool res_primitive = false;
  for (auto &c : cc->getObstacles()) {
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

bool CollisionCheckerTest::test_collide_obstacles(
    CollisionObjectConstPtr co, const CollisionChecker *cc,
    std::vector<CollisionObjectConstPtr> &missed_obstacles,
    std::vector<CollisionObjectConstPtr> &missed_obstacles_primit) {

  std::vector<int> failure_export;

  std::vector<CollisionObjectConstPtr> obstacles_cc;

  bool res_cc = cc->collide(co, obstacles_cc, false, false);

  std::vector<CollisionObjectConstPtr> obstacles_primit;
  bool res_primitive = false;
  for (auto &c : cc->getObstacles()) {
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
      *sg_fcl, *co_fcl);

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
      *sg1_fcl, *sg2_fcl);

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

int aabb_tolerance_check(CollisionObjectConstPtr obj1, CollisionObjectConstPtr obj2, bool& tolerance_result, double& distance) {

	if (obj1->getCollisionObjectClass() == CollisionObjectClass::OBJ_CLASS_SHAPE &&
		obj2->getCollisionObjectClass() == CollisionObjectClass::OBJ_CLASS_SHAPE) {
		const auto aabb_A = obj1->getAABB();
		const auto aabb_B = obj2->getAABB();

		const auto A_min = aabb_A->min();
		const auto A_max = aabb_A->max();

		const auto B_min = aabb_B->min();
		const auto B_max = aabb_B->max();

		const auto delta1 = A_min - B_max;
		const auto delta2 = B_min - A_max;

		const auto delta3 = A_min - B_min;
		const auto delta4 = B_min - A_min;


		const auto u = Eigen::Vector2d {std::max(0., delta1.x()), std::max(0., delta1.y())};
		const auto v = Eigen::Vector2d {std::max(0., delta2.x()), std::max(0., delta2.y())};


		double tol1 = 0.;
		double tol2 = 0.;

		if(delta1.x() < 0. and delta1.y() < 0. and delta3.x() > 0. and delta3.y() > 0.)
			tol1 = tol1 + fabs(std::max(delta1.x(), delta1.y()));

		if(delta2.x() < 0. and delta2.y() < 0. and delta4.x() > 0. and delta4.y() > 0.)
			tol2 = tol2 + fabs(std::max(delta2.x(), delta2.y()));
		distance = (u.norm() + v.norm() + tol1 + tol2);
		tolerance_result = distance < AABB_TOLERANCE_THRESHOLD;
		return 0;
	}
	else {
		return -1;
	}
}

bool is_borderline_case_sg_support(CollisionObjectConstPtr obj1,
		CollisionObjectConstPtr obj2) {
	bool tolerance_result = false;
	double distance = 0.;
	if (obj1->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP &&
			obj2->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP) {
		auto sg1 = static_cast<const ShapeGroup*>(obj1.get());
		auto sg2 = static_cast<const ShapeGroup*>(obj2.get());
		for (auto shape1: sg1->unpack()) {
			for (auto shape2: sg2->unpack()){
				if (!aabb_tolerance_check(shape1,shape2, tolerance_result, distance) && tolerance_result) {
					return true;
				}
			}
		}
		return false;
	} else if (obj1->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP &&
			obj2->getCollisionObjectClass() == OBJ_CLASS_SHAPE) {
		auto sg1 = static_cast<const ShapeGroup*>(obj1.get());
		for (auto shape: sg1->unpack()) {
			if (!aabb_tolerance_check(shape,obj2, tolerance_result, distance) && tolerance_result) {
				return true;
			}
		}
		return false;
	} else if (obj2->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP &&
			obj1->getCollisionObjectClass() == OBJ_CLASS_SHAPE) {
		return is_borderline_case_sg_support(obj2, obj1);
	} else if (obj1->getCollisionObjectClass() == OBJ_CLASS_SHAPE &&
			obj2->getCollisionObjectClass() == OBJ_CLASS_SHAPE) {
		if (!aabb_tolerance_check(obj1, obj2, tolerance_result, distance) && tolerance_result) {
			return true;
		}
		return false;
	}
	throw;
}

bool is_borderline_case_tvobstacle_support(CollisionObjectConstPtr obj,
		CollisionObjectConstPtr co) {
	bool result = true;
	if (obj->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE
			|| co->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
		bool tvgroup_tolerance_passed = true;
		int time_start_idx = 0, time_end_idx = 0;
		const TimeVariantCollisionObject *obj_tv = 0;
		const TimeVariantCollisionObject *co_tv = 0;
		if (co->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
			co_tv = static_cast<const TimeVariantCollisionObject*>(co.get());
		}
		if (obj->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
			obj_tv = static_cast<const TimeVariantCollisionObject*>(obj.get());
		}
		if (obj_tv && co_tv) {
			time_start_idx = std::min(obj_tv->time_start_idx(),
					co_tv->time_start_idx());
			time_end_idx = std::max(obj_tv->time_end_idx(),
					co_tv->time_end_idx());
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
		for (int time_idx = time_start_idx; time_idx < time_end_idx;
				time_idx++) {
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
			bool ret = is_borderline_case_sg_support(obj1, obj2);
			if (ret) {
				tvgroup_tolerance_passed = false;
				break;
			}
		}
		if (!tvgroup_tolerance_passed) {
			result = true;
		} else {
			result = false;
		}
	} else {
		bool tol_res = false;
		double dist = 0;
		bool ret = is_borderline_case_sg_support(obj, co);
		result=ret;
	}
	return result;
}

bool test_tolerance_check(CollisionObjectConstPtr co,
		const CollisionChecker *cc) {
	bool result = true;
	bool true_Failure = false;

	for (auto &obj : cc->getObstacles()) {
		result = is_borderline_case_tvobstacle_support(obj, co);
		if (!result) {
			true_Failure = true;
			break;
		}
	}
	return true_Failure;
}

bool ShapeGroupTest::test_overlap_map(
    const ShapeGroup *sg1, const ShapeGroup *sg2,
    std::vector<std::pair<int, int>> *missed_col_naive_out) {
  std::vector<std::pair<int, int>> res = sg1->overlap(*sg2);
  std::vector<collision::ShapeConstPtr> obstacles1 = sg1->unpack();
  std::vector<collision::ShapeConstPtr> obstacles2 = sg2->unpack();

  std::vector<std::pair<int, int>> overlap_res = sg1->overlap(*sg2);
  std::vector<std::pair<int, int>> overlap_res2;

  for (int cc1 = 0; cc1 < obstacles1.size(); cc1++) {
    for (int cc2 = 0; cc2 < obstacles2.size(); cc2++) {
      if (obstacles1[cc1]->collide(*obstacles2[cc2],
                                   CollisionRequest(COL_FCL))) {
        overlap_res2.emplace_back(cc1, cc2);
      }
    }
  }

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
          aabb_tolerance_check(obstacles1[miss.first], obstacles2[miss.second],
                         tolerance_result, distance);

      if (!tol_res && !tolerance_result) {
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
          aabb_tolerance_check(obstacles1[miss.first], obstacles2[miss.second],
                         tolerance_result, distance);
      if (!tol_res && !tolerance_result) {
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
  if (!res) {
    bool local_failure1 = false;
    for (auto el : sg->unpack()) {
      bool tolerance_result = false;
      double distance = 0;
		if (!aabb_tolerance_check(co, el, tolerance_result, distance) && !tolerance_result) {
			local_failure1 = true;
			break;
		}
    }
    if (local_failure1) {
    	return false;
    }
  }
  return true;
}

bool ShapeGroupTest::run_test_collide(const ShapeGroup *sg1,
                                      const ShapeGroup *sg2) {
  bool res = test_collide(sg1, sg2);

  bool res2 = test_overlap_map(sg1, sg2);
  if (!res || !res2)
  {
    res = test_collide(sg1, sg2);
    res2 = test_overlap_map(sg1, sg2);
  }
  return res && res2;
}

bool CollisionCheckerTest::run_test_collide(CollisionObjectConstPtr co,
                                            const CollisionChecker *cc) {
  return test_collide(co, cc);
}

bool CollisionCheckerTest::run_test_collide_obstacle(
    CollisionObjectConstPtr co, const CollisionChecker *cc) {
  return test_collide_obstacle(co, cc);
}

bool CollisionCheckerTest::run_test_collide_obstacles(
    CollisionObjectConstPtr co, const CollisionChecker *cc) {
  std::vector<CollisionObjectConstPtr> missed_obstacles;
  std::vector<CollisionObjectConstPtr> missed_obstacles_primit;
  bool res =
      test_collide_obstacles(co, cc, missed_obstacles, missed_obstacles_primit);
  if (!res) {
    CollisionChecker cc2;
    for (auto &obst : missed_obstacles) {
      bool naive_collide = obst->collide(*co, CollisionRequest(COL_DEFAULT));
      bool non_naive_collide = obst->collide(*co, CollisionRequest(COL_FCL));
      std::cout << "[CollisionCheckerTest::run_test_collide_obstacles]: CollisionChecker broadphase missed an obstacle as "
    		  "compared to using bruteforce collision checking. Checking if there are objects with AABBs that intersect just on the border.\n";
      cc2.addCollisionObject(obst);
    }
    for (auto &obst : missed_obstacles_primit) {
      bool naive_collide = obst->collide(*co, CollisionRequest(COL_DEFAULT));
      bool non_naive_collide = obst->collide(*co, CollisionRequest(COL_FCL));
      std::cout << "[CollisionCheckerTest::run_test_collide_obstacles]: Bruteforce collision check missed an obstacle "
    		  "as compared to using FCL broadphase. Checking if there are objects with AABBs that intersect just on the border.\n";
    }
    bool tolerance_result = test_tolerance_check(co, &cc2);
    if (tolerance_result) {
      // there are no objects with AABBs very close to each other, intersecting just on the border
      return false;
    }
  }
  return true;
}

}  // namespace test
}  // namespace collision
