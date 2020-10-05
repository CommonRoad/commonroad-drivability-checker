#include "collision/solvers/collision_solver_fcl.h"
#include "collision/solvers/collision_solver_primitive.h"
#include "collision/solvers/collision_solvers.h"
#include "collision/time_variant_collision_object.h"

namespace collision {
namespace solvers {
namespace solverDefault {

std::size_t collide_tvobst_obj(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  static solvers::DefaultSolver solver;
  static CollisionFunctionMatrix matr_default(&solver);
  const TimeVariantCollisionObject &tvobst(
      static_cast<const TimeVariantCollisionObject &>(obj1));
  int time_idx_start = tvobst.time_start_idx();
  int time_idx_end = tvobst.time_end_idx();
  for (int cc1 = time_idx_start; cc1 <= time_idx_end; cc1++) {
    CollisionObjectConstPtr cur_obj = tvobst.getObstacleAtTime(cc1);

    if (!cur_obj.get()) {
      continue;
    }

    if (cur_obj->getCollisionObjectClass() ==
        CollisionObjectClass::OBJ_CLASS_TVOBSTACLE) {
      return -1;
    }

    collide_bool_func_t func = matr_default.getSolverBoolFunction(
        cur_obj->getCollisionObjectType(), obj2.getCollisionObjectType());
    if (!func) {
      return -1;
    }
    func(*cur_obj, obj2, res, req);
    if (res.collides()) return 0;
  }
  return 0;
}

std::size_t collide_obj_tvobst(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  return collide_tvobst_obj(obj2, obj1, res, req);
}

std::size_t collide_tvobst_tvobst(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req) {
  static solvers::DefaultSolver solver;
  static CollisionFunctionMatrix matr_default(&solver);
  const TimeVariantCollisionObject &tvobst1(
      static_cast<const TimeVariantCollisionObject &>(obj1));
  const TimeVariantCollisionObject &tvobst2(
      static_cast<const TimeVariantCollisionObject &>(obj2));
  int time_idx_start =
      std::min(tvobst1.time_start_idx(), tvobst2.time_start_idx());
  int time_idx_end = std::max(tvobst1.time_end_idx(), tvobst2.time_end_idx());

  for (int cc1 = time_idx_start; cc1 <= time_idx_end; cc1++) {
    CollisionObjectConstPtr obst_1 = tvobst1.getObstacleAtTime(cc1);
    CollisionObjectConstPtr obst_2 = tvobst2.getObstacleAtTime(cc1);

    if (!obst_1.get() || !obst_2.get()) {
      continue;
    }
    if (obst_1->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE ||
        obst_2->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE) {
      return -1;
    }

    collide_bool_func_t func = matr_default.getSolverBoolFunction(
        obst_1->getCollisionObjectType(), obst_2->getCollisionObjectType());
    if (!func) {
      return -1;
    }
    func(*obst_1, *obst_2, res, req);
    if (res.collides()) return 0;
  }
  return 0;
}
}  // namespace solverDefault

CollisionFunctionMatrix::CollisionFunctionMatrix(
    const solvers::DefaultSolver *solver) {
  memset(m_collide_bool_function, 0,
         COL_OBJECT_TYPES_COUNT * COL_OBJECT_TYPES_COUNT);
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_AABB_BOX] =
      solvers::solverPrimitive::collide_aabb_aabb;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_OBB_BOX] =
      solvers::solverPrimitive::collide_aabb_obb;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POINT] =
      solvers::solverPrimitive::collide_aabb_point;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_AABB_BOX] =
      solvers::solverPrimitive::collide_obb_aabb;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_OBB_BOX] =
      solvers::solverPrimitive::collide_obb_obb;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_POINT] =
      solvers::solverPrimitive::collide_sphere_point;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_AABB_BOX] =
      solvers::solverPrimitive::collide_point_aabb;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_SPHERE] =
      solvers::solverPrimitive::collide_point_sphere;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_POINT] =
      solvers::solverPrimitive::collide_point_point;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_shape_group_obj;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_shape_group_shape_group;
  m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_tvobst_obj;
  m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_tvobst_tvobst;
}
}  // namespace solvers
}  // namespace collision
