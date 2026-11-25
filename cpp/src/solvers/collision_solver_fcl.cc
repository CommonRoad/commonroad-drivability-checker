#include "collision/solvers/collision_solver_fcl.h"

#include "collision/collision_object_types.h"
#include "collision/solvers/collision_solvers.h"
#include "collision/solvers/fcl/fcl_collision_queries.h"
#include "collision/solvers/fcl/fcl_helpers.h"

namespace collision {

namespace solvers {
namespace solverFCL {

/*!
 \brief Helper function to check for a collision between two Shapes
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_obj_obj(const CollisionObject &obj1,
                            const CollisionObject &obj2, CollisionResult &res,
                            const CollisionRequest &req) {
  const FCLCollisionObject *obj1_fcl = get_fcl_object_ptr(&obj1);
  const FCLCollisionObject *obj2_fcl = get_fcl_object_ptr(&obj2);
  if (!obj1_fcl || !obj2_fcl) return -1;

  res.set_result(fcl_primitive_queries::fcl_generic_collisionDetection(
      *obj1_fcl, *obj2_fcl));
  return 0;
}

/*!
 \brief Helper function to check for a collision between a ShapeGroup and a Shape
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_shape_group_obj(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req) {
  const FCLCollisionObjectGroup *obj1_fcl = get_fcl_object_group_ptr(&obj1);
  const FCLCollisionObject *obj2_fcl = get_fcl_object_ptr(&obj2);
  if (!obj1_fcl || !obj2_fcl) return -1;
  res.set_result(fcl_primitive_queries::fcl_generic_collisionDetection(
      *obj1_fcl, *obj2_fcl));
  return 0;
}

/*!
 \brief Helper function to check for a collision between a Shape and a ShapeGroup
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_obj_shape_group(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req) {
  return collide_shape_group_obj(obj2, obj1, res, req);
}

/*!
 \brief Helper function to check for a collision between two ShapeGroups
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_shape_group_shape_group(const CollisionObject &obj1,
                                            const CollisionObject &obj2,
                                            CollisionResult &res,
                                            const CollisionRequest &req) {
  const FCLCollisionObjectGroup *obj1_fcl = get_fcl_object_group_ptr(&obj1);
  const FCLCollisionObjectGroup *obj2_fcl = get_fcl_object_group_ptr(&obj2);

  if (!obj1_fcl || !obj2_fcl) return -1;
  res.set_result(fcl_primitive_queries::fcl_generic_collisionDetection(
      *obj1_fcl, *obj2_fcl));
  return 0;
}

/*!
 \brief Helper function to check for a collision between a TimeVariantCollisionObject and a non-time-variant CollisionObject
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_tvobst_obj(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  static solvers::FCLSolver solver;
  static CollisionFunctionMatrix matr_fcl(&solver);
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

    collide_bool_func_t func = matr_fcl.getSolverBoolFunction(
        cur_obj->getCollisionObjectType(), obj2.getCollisionObjectType());
    if (!func) {
      return -1;
    }
    func(*cur_obj, obj2, res, req);
    if (res.collides()) return 0;
  }
  return 0;
}

/*!
 \brief Helper function to check for a collision between a non-time-variant CollisionObject and a TimeVariantCollisionObject
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_obj_tvobst(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  return collide_tvobst_obj(obj2, obj1, res, req);
}

/*!
 \brief Helper function to check for a collision between two time-variant collision objects
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

std::size_t collide_tvobst_tvobst(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req) {
  static solvers::FCLSolver solver;
  static CollisionFunctionMatrix matr_fcl(&solver);
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

    collide_bool_func_t func = matr_fcl.getSolverBoolFunction(
        obst_1->getCollisionObjectType(), obst_2->getCollisionObjectType());
    if (!func) {
      return -1;
    }
    func(*obst_1, *obst_2, res, req);
    if (res.collides()) return 0;
  }
  return 0;
}

}  // namespace solverFCL

/*!
 \brief Constructor that fills in pointers to functions to which the collision queries are to be dispatched.
*/

CollisionFunctionMatrix::CollisionFunctionMatrix(
    const solvers::FCLSolver *solver) {
  memset(m_collide_bool_function, 0,
         COL_OBJECT_TYPES_COUNT * COL_OBJECT_TYPES_COUNT);
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POLYGON] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SHAPEGROUP] =
      solvers::solverFCL::collide_obj_shape_group;
  m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TVOBSTACLE] =
      solvers::solverFCL::collide_obj_tvobst;

  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_AABB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
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
      solvers::solverFCL::collide_obj_obj;
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
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_OBB_BOX] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_SPHERE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_TRIANGLE] =
      solvers::solverFCL::collide_obj_obj;
  m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_POINT] =
      solvers::solverFCL::collide_obj_obj;
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
