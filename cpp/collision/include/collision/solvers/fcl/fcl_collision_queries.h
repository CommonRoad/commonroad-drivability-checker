#ifndef CPP_COLLISION_FCL_FCL_PRIMITIVE_COLLISION_TESTS_H_
#define CPP_COLLISION_FCL_FCL_PRIMITIVE_COLLISION_TESTS_H_

#include "collision/application.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include "collision/solvers/fcl/fcl_helpers.h"

#include <Eigen/Dense>

#include "collision/narrowphase/point.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/shape.h"
#include "collision/narrowphase/sphere.h"
#include "collision/narrowphase/triangle.h"

#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"

#include "collision/solvers/fcl/fcl_collision_requests.h"
#include "collision/solvers/primitive_collision_queries.h"

namespace collision {
namespace solvers {
namespace solverFCL {

namespace fcl_primitive_queries {

inline bool fcl_testAABB(CollisionObjectConstPtr obj,
                         const collision::RectangleAABB &rect) {
  SolverEntity_FCL *obj_entity;
  FCL_COLLISION_ENTITY_TYPE enttype =
      get_object_fcl_entity_type(obj, obj_entity);
  if (enttype == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    const FCLCollisionObject *obj_fcl =
        static_cast<const FCLCollisionObject *>(obj_entity);

    fcl::CollisionObject<double> *fcl_obj =
        obj_fcl->getCollisionObject_fcl().get();
    if (!fcl_obj) {
      throw;
    }
    const fcl::AABB<double> &aabb1 = fcl_obj->getAABB();

    Eigen::Vector2d min_ = rect.min();
    Eigen::Vector2d max_ = rect.max();

    fcl::Vector3d min_3d(min_[0], min_[1], 0);
    fcl::Vector3d max_3d(max_[0], max_[1], 0);

    fcl::AABB<double> aabb2(min_3d, max_3d);
    return aabb1.overlap(aabb2);
  }
  return true;
}

using CollisionGeometryPtr_t =
    std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>>;

inline fcl::Transform3<FCL_PRECISION> fcl_get_3d_translation(
    const Eigen::Vector2d &pos) {
  return fcl::Transform3<FCL_PRECISION>(fcl::Translation3<FCL_PRECISION>(
      fcl::Vector3<FCL_PRECISION>(pos.x(), pos.y(), 0)));
}
inline bool fcl_collide(const fcl::CollisionObject<FCL_PRECISION> &object_A,
                        const fcl::CollisionObject<FCL_PRECISION> &object_B) {
  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false);  // Disable finding of contact points
  fcl::CollisionResult<FCL_PRECISION> collisionResult;
  collisionRequest.enable_cost = false;
  collisionRequest.gjk_solver_type = FCL_SOLVER_TYPE;
  fcl::collide(&object_A, &object_B, collisionRequest, collisionResult);
  return collisionResult.isCollision();
}

inline bool fcl_generic_collisionDetection(const FCLCollisionObject &obj_first,
                                           const FCLCollisionObject &obj_second)

{
  std::shared_ptr<fcl::CollisionObject<double>> col_obj_1;

  col_obj_1 = obj_first.getCollisionObject_fcl();

  std::shared_ptr<fcl::CollisionObject<double>> col_obj_2;
  col_obj_2 = obj_second.getCollisionObject_fcl();

  return fcl_collide(*col_obj_1, *col_obj_2);
}
#if ENABLE_COLLISION_TESTS
// TODO: throw exception on malloc failure
inline bool fcl_generic_collisionDetection(
    const FCLCollisionObjectGroup &group_first,
    const FCLCollisionObject &obj_second, bool test_enable = true) {
  if (test_enable) {
    collision::test::ShapeGroupTest sg_test;
    sg_test.run_test_collide(
        obj_second.getParent()->shared_from_this(),
        static_cast<const ShapeGroup *>(group_first.getParent()));
#if ENABLE_COLLISION_TESTS_NARROWPHASE
    sg_test.run_test_narrowphase(
        obj_second.getParent()->shared_from_this(),
        static_cast<const ShapeGroup *>(group_first.getParent()));
#endif
  }
#else
inline bool fcl_generic_collisionDetection(
    const FCLCollisionObjectGroup &group_first,
    const FCLCollisionObject &obj_second) {
#endif

  bool result = false;
  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr;
  group_first.getManager_fcl(mngr);
  fcl::CollisionResult<FCL_PRECISION> collisionResult;
  CollisionData<FCL_PRECISION> self_data_obstacles1;
  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false);  // Disable finding of contact points
  collisionRequest.enable_cost = false;
  collisionRequest.gjk_solver_type = FCL_SOLVER_TYPE;
  self_data_obstacles1.request = collisionRequest;
  std::shared_ptr<fcl::CollisionObject<double>> col_obj_1;
  col_obj_1 = obj_second.getCollisionObject_fcl();

  mngr->collide(col_obj_1.get(), &self_data_obstacles1,
                defaultCollisionFunction<FCL_PRECISION>);
  result = self_data_obstacles1.result.isCollision();
  return result;
}

inline int fcl_group_overlap(
    const FCLCollisionObjectGroup &group_first,
    const FCLCollisionObjectGroup &group_second,
    std::vector<std::pair<int, int>> &retlist,
    std::vector<std::pair<int, int>> *missed_col_naive_in = 0) {
  // bool result=false;

  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr1;
  group_first.getManager_fcl(mngr1);

  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr2;
  group_second.getManager_fcl(mngr2);

  fcl::CollisionResult<FCL_PRECISION> collisionResult;

  CollisionRequestDataOverlap rdata(group_first.getContainer(),
                                    group_second.getContainer());

  CollisionDataOverlap<FCL_PRECISION> self_data_obstacles(rdata);
  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false);  // Disable finding of contact points
  collisionRequest.gjk_solver_type = FCL_SOLVER_TYPE;
  collisionRequest.enable_cost = false;

  self_data_obstacles.request = collisionRequest;

  mngr1->collide(mngr2, &self_data_obstacles,
                 defaultCollisionFunctionOverlap<FCL_PRECISION>);
  auto res = self_data_obstacles.self_reqData.getRequestResultPairs();
  for (auto el : res) {
    retlist.push_back(el);
  }

  return 0;
}

#if ENABLE_COLLISION_TESTS
inline bool fcl_generic_collisionDetection(
    const FCLCollisionObjectGroup &group_first,
    const FCLCollisionObjectGroup &group_second, bool enable_test = true) {
  if (enable_test) {
    collision::test::ShapeGroupTest sg_test;
    const ShapeGroup *gr_first =
        static_cast<const ShapeGroup *>(group_first.getParent());
    const ShapeGroup *gr_second =
        static_cast<const ShapeGroup *>(group_second.getParent());
    sg_test.run_test_collide(gr_first, gr_second);
#if ENABLE_COLLISION_TESTS_NARROWPHASE
    sg_test.run_test_narrowphase(gr_first, gr_second);
#endif
  }
#else
inline bool fcl_generic_collisionDetection(
    const FCLCollisionObjectGroup &group_first,
    const FCLCollisionObjectGroup &group_second) {
#endif
  bool result = false;
  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr1;
  group_first.getManager_fcl(mngr1);

  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr2;
  group_second.getManager_fcl(mngr2);

  fcl::CollisionResult<FCL_PRECISION> collisionResult;
  CollisionData<FCL_PRECISION> self_data_obstacles;

  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false);  // Disable finding of contact points
  collisionRequest.gjk_solver_type = FCL_SOLVER_TYPE;
  collisionRequest.enable_cost = false;

  self_data_obstacles.request = collisionRequest;

  mngr1->collide(mngr2, &self_data_obstacles,
                 defaultCollisionFunction<FCL_PRECISION>);
  result = self_data_obstacles.result.isCollision();
  return result;
}

inline bool fcl_collisionDetection(const Point &point,
                                   const RectangleAABB &aabb) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)point,
                                        (const FCLCollisionObject &)aabb);
}

inline bool fcl_collisionDetection(const Point &point,
                                   const RectangleOBB &obb) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)point,
                                        (const FCLCollisionObject &)obb);
}
inline bool fcl_collisionDetection(const Point &point, const Sphere &sphere) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)point,
                                        (const FCLCollisionObject &)sphere);
}

inline bool fcl_collisionDetection(const Point &point1, const Point &point2) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)point1,
                                        (const FCLCollisionObject &)point2);
}

inline bool fcl_collisionDetection(const Point &point,
                                   const Triangle &triangle) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)point,
                                        (const FCLCollisionObject &)triangle);
}

inline bool fcl_collisionDetection(const Polygon &polyg1,
                                   const Polygon &polyg2) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)polyg1,
                                        (const FCLCollisionObject &)polyg2);
}

inline bool fcl_collisionDetection(const Polygon &polyg, const Shape &shape) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)polyg,
                                        (const FCLCollisionObject &)shape);
}

inline bool fcl_collisionDetection(const ShapeGroup &shg,
                                   const Polygon &polyg) {
  return fcl_generic_collisionDetection((const FCLCollisionObjectGroup &)shg,
                                        (const FCLCollisionObject &)polyg);
}

inline bool fcl_collisionDetection(const Sphere &sphere_first,
                                   const Sphere &sphere_second) {
  return fcl_generic_collisionDetection(
      (const FCLCollisionObject &)sphere_first,
      (const FCLCollisionObject &)sphere_second);
}
inline bool fcl_collisionDetection(const RectangleAABB &aabb_first,
                                   const RectangleAABB &aabb_second) {
  return fcl_generic_collisionDetection(
      (const FCLCollisionObject &)aabb_first,
      (const FCLCollisionObject &)aabb_second);
}

inline bool fcl_collisionDetection(const RectangleAABB &aabb,
                                   const RectangleOBB &obb) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)aabb,
                                        (const FCLCollisionObject &)obb);
}

inline bool fcl_collisionDetection(const ShapeGroup &shg, const Shape &shape) {
  return fcl_generic_collisionDetection((const FCLCollisionObjectGroup &)shg,
                                        (const FCLCollisionObject &)shape);
}

inline bool fcl_collisionDetection(const ShapeGroup &shg1,
                                   const ShapeGroup &shg2) {
  return fcl_generic_collisionDetection((const FCLCollisionObjectGroup &)shg1,
                                        (const FCLCollisionObjectGroup &)shg2);
}

inline bool fcl_collisionDetection(const RectangleAABB &aabb,
                                   const Sphere &sphere) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)aabb,
                                        (const FCLCollisionObject &)sphere);
}
inline bool fcl_collisionDetection(const RectangleOBB &obb_first,
                                   const RectangleOBB &obb_second) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)obb_first,
                                        (const FCLCollisionObject &)obb_second);
}
inline bool fcl_collisionDetection(const RectangleOBB &obb,
                                   const Sphere &sphere) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)obb,
                                        (const FCLCollisionObject &)sphere);
}

inline bool fcl_collisionDetection(const Sphere &sphere,
                                   const Triangle &triangle) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)sphere,
                                        (const FCLCollisionObject &)triangle);
}
inline bool fcl_collisionDetection(const Triangle &triangle_a,
                                   const Triangle &triangle_b) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)triangle_a,
                                        (const FCLCollisionObject &)triangle_b);
}
inline bool fcl_collisionDetection(const RectangleOBB &obb,
                                   const Triangle &triangle) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)obb,
                                        (const FCLCollisionObject &)triangle);
}
inline bool fcl_collisionDetection(const RectangleAABB &aabb,
                                   const Triangle &triangle) {
  return fcl_generic_collisionDetection((const FCLCollisionObject &)aabb,
                                        (const FCLCollisionObject &)triangle);
}

}  // namespace fcl_primitive_queries
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_FCL_FCL_PRIMITIVE_COLLISION_TESTS_H_ */
