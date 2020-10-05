#include "collision/solvers/collision_solver_primitive.h"

#include "collision/solvers/primitive_collision_queries.h"

namespace collision {

namespace solvers {
namespace solverPrimitive {
int not_implemented_error(const CollisionObject &obj1,
                          const CollisionObject &obj2,
                          const char *solver_name) {
  std::cout << std::string("collision function between object of types ") +
                   std::to_string(obj1.getCollisionObjectType()) +
                   std::string(" and ") +
                   std::to_string(obj2.getCollisionObjectType()) +
                   std::string("is not implemented in the solver ") +
                   std::string(solver_name)
            << std::endl;
  throw;
  return 0;
}
// critical not to make mistakes
std::size_t collide_aabb_aabb(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const RectangleAABB &>(obj1),
      static_cast<const RectangleAABB &>(obj2)));
  return 0;
}
std::size_t collide_aabb_obb(const CollisionObject &obj1,
                             const CollisionObject &obj2, CollisionResult &res,
                             const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const RectangleAABB &>(obj1),
      static_cast<const RectangleOBB &>(obj2)));
  return 0;
}
std::size_t collide_aabb_sphere(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleAABB&>(obj1), static_cast<const Sphere&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_aabb_triangle(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleAABB&>(obj1), static_cast<const Triangle&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_aabb_point(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const Point &>(obj2),
      static_cast<const RectangleAABB &>(obj1)));
  return 0;
}
std::size_t collide_obb_aabb(const CollisionObject &obj1,
                             const CollisionObject &obj2, CollisionResult &res,
                             const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const RectangleAABB &>(obj2),
      static_cast<const RectangleOBB &>(obj1)));
  return 0;
}
std::size_t collide_obb_obb(const CollisionObject &obj1,
                            const CollisionObject &obj2, CollisionResult &res,
                            const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const RectangleOBB &>(obj1),
      static_cast<const RectangleOBB &>(obj2)));
  return 0;
}
std::size_t collide_obb_sphere(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleOBB&>(obj1), static_cast<const Sphere&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_obb_triangle(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleOBB&>(obj1), static_cast<const Triangle&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_obb_point(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Point&>(obj2), static_cast<const RectangleOBB&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_sphere_aabb(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleAABB&>(obj2), static_cast<const Sphere&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}

std::size_t collide_sphere_obb(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleOBB&>(obj2), static_cast<const Sphere&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}

std::size_t collide_sphere_sphere(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Sphere&>(obj1), static_cast<const Sphere&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}

std::size_t collide_sphere_triangle(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Sphere&>(obj1), static_cast<const Triangle&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}

std::size_t collide_sphere_point(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const Point &>(obj2), static_cast<const Sphere &>(obj1)));
  return 0;
}
std::size_t collide_triangle_aabb(const CollisionObject &obj1,
                                  const CollisionObject &obj2,
                                  CollisionResult &res,
                                  const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleAABB&>(obj2), static_cast<const Triangle&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_triangle_obb(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // RectangleOBB&>(obj2), static_cast<const Triangle&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_triangle_sphere(const CollisionObject &obj1,
                                    const CollisionObject &obj2,
                                    CollisionResult &res,
                                    const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Sphere&>(obj2), static_cast<const Triangle&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_triangle_triangle(const CollisionObject &obj1,
                                      const CollisionObject &obj2,
                                      CollisionResult &res,
                                      const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Triangle&>(obj1), static_cast<const Triangle&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_triangle_point(const CollisionObject &obj1,
                                   const CollisionObject &obj2,
                                   CollisionResult &res,
                                   const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Point&>(obj2), static_cast<const Triangle&>(obj1)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}

std::size_t collide_point_aabb(const CollisionObject &obj1,
                               const CollisionObject &obj2,
                               CollisionResult &res,
                               const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const Point &>(obj1),
      static_cast<const RectangleAABB &>(obj2)));
  return 0;
}
std::size_t collide_point_obb(const CollisionObject &obj1,
                              const CollisionObject &obj2, CollisionResult &res,
                              const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Point&>(obj1), static_cast<const RectangleOBB&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_point_sphere(const CollisionObject &obj1,
                                 const CollisionObject &obj2,
                                 CollisionResult &res,
                                 const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const Point &>(obj1), static_cast<const Sphere &>(obj2)));
  return 0;
}
std::size_t collide_point_triangle(const CollisionObject &obj1,
                                   const CollisionObject &obj2,
                                   CollisionResult &res,
                                   const CollisionRequest &req) {
  // res.set_result(primitive_queries::collisionDetection(static_cast<const
  // Point&>(obj1), static_cast<const Triangle&>(obj2)));
  not_implemented_error(obj1, obj2, "primitiveSolver");
  return -1;
}
std::size_t collide_point_point(const CollisionObject &obj1,
                                const CollisionObject &obj2,
                                CollisionResult &res,
                                const CollisionRequest &req) {
  res.set_result(primitive_queries::collisionDetection(
      static_cast<const Point &>(obj1), static_cast<const Point &>(obj2)));
  return 0;
}

/*
std::size_t collide_polygon_obj(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        std::vector<TriangleConstPtr> mesh_triangles = (static_cast<const
Polygon&> (obj1)).getTriangleMesh(); for (auto& obj : mesh_triangles)
        {
                collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(OBJ_TYPE_TRIANGLE,
obj2.getCollisionObjectType()); if (!func)
                {
                        return -1;
                }
                func(*obj, obj2, res, req);
                if (res.collides())
                        return 0;
        }
        return 0;
}

std::size_t collide_polygon_polygon(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        std::vector<TriangleConstPtr> mesh_triangles_1 = (static_cast<const
Polygon&> (obj1)).getTriangleMesh(); std::vector<TriangleConstPtr>
mesh_triangles_2 = (static_cast<const Polygon&> (obj2)).getTriangleMesh(); for
(auto& obj1 : mesh_triangles_1)
        {
                for (auto& obj2 : mesh_triangles_2)
                {
                        collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(OBJ_TYPE_TRIANGLE, OBJ_TYPE_TRIANGLE); if
(!func)
                        {
                                return -1;
                        }
                        func(*obj1, *obj2, res, req);
                        if (res.collides())
                                return 0;
                }
        }
        return 0;
}

std::size_t collide_obj_polygon(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        return collide_polygon_obj(obj2, obj1, res, req);
}

std::size_t collide_shape_group_obj(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        std::vector<ShapeConstPtr> sg_objects = (static_cast<const ShapeGroup&>
(obj1)).unpack(); for (auto& obj : sg_objects)
        {
                collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(obj->getCollisionObjectType(),
obj2.getCollisionObjectType()); if (!func)
                {
                        return -1;
                }
                func(*obj, obj2, res, req);
                if (res.collides())
                        return 0;
        }
        return 0;
}

std::size_t collide_obj_shape_group(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        return collide_shape_group_obj(obj2, obj1, res, req);
}

std::size_t collide_shape_group_shape_group(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        std::vector<ShapeConstPtr> sg_objects_1 = (static_cast<const
ShapeGroup&> (obj1)).unpack(); std::vector<ShapeConstPtr> sg_objects_2 =
(static_cast<const ShapeGroup&> (obj2)).unpack();

        for (auto& obj_1 : sg_objects_1)
        {
                for (auto& obj_2 : sg_objects_2)
                {

                        collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(obj_1->getCollisionObjectType(),
obj_2->getCollisionObjectType()); if (!func)
                        {
                                return -1;
                        }
                        func(*obj_1, *obj_2, res, req);
                        if (res.collides())
                                return 0;
                }
        }
        return 0;
}

std::size_t collide_tvobst_obj(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        const TimeVariantCollisionObject& tvobst(static_cast<const
TimeVariantCollisionObject&>(obj1)); int time_idx_start =
tvobst.time_start_idx(); int time_idx_end = tvobst.time_end_idx(); for (int cc1
= time_idx_start; cc1 <= time_idx_end; cc1++)
        {
                CollisionObjectConstPtr cur_obj = tvobst.getObstacleAtTime(cc1);

                if (!cur_obj.get())
                {
                        continue;
                }

                if (cur_obj->getCollisionObjectClass() ==
CollisionObjectClass::OBJ_CLASS_TVOBSTACLE)
                {
                        return -1;
                }

                collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(cur_obj->getCollisionObjectType(),
obj2.getCollisionObjectType()); if (!func)
                {
                        return -1;
                }
                func(*cur_obj, obj2, res, req);
                if (res.collides())
                        return 0;
        }
        return 0;
}

std::size_t collide_obj_tvobst(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        return collide_tvobst_obj(obj2, obj1, res, req);
}


std::size_t collide_tvobst_tvobst(const CollisionObject& obj1, const
CollisionObject& obj2, CollisionResult& res, const CollisionRequest& req)
{
        static solvers::PrimitiveSolver solver;
        static CollisionFunctionMatrix matr_primitive(&solver);
        const TimeVariantCollisionObject& tvobst1(static_cast<const
TimeVariantCollisionObject&>(obj1)); const TimeVariantCollisionObject&
tvobst2(static_cast<const TimeVariantCollisionObject&>(obj2)); int
time_idx_start = std::min(tvobst1.time_start_idx(), tvobst2.time_start_idx());
        int time_idx_end = std::max(tvobst1.time_end_idx(),
tvobst2.time_end_idx());

        for (int cc1 = time_idx_start; cc1 <= time_idx_end; cc1++)
        {
                CollisionObjectConstPtr obst_1 = tvobst1.getObstacleAtTime(cc1);
                CollisionObjectConstPtr obst_2 = tvobst2.getObstacleAtTime(cc1);

                if (!obst_1.get() || !obst_2.get())
                {
                        continue;
                }
                if (obst_1->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE ||
obst_2->getCollisionObjectClass() == OBJ_CLASS_TVOBSTACLE)
                {
                        return -1;
                }

                collide_bool_func_t func =
matr_primitive.getSolverBoolFunction(obst_1->getCollisionObjectType(),
obst_2->getCollisionObjectType()); if (!func)
                {
                        return -1;
                }
                func(*obst_1, *obst_2, res, req);
                if (res.collides())
                        return 0;
        }
        return 0;
}
*/

}  // namespace solverPrimitive
/*

CollisionFunctionMatrix::CollisionFunctionMatrix(const solvers::PrimitiveSolver*
solver)
{
        memset(m_collide_bool_function, 0,
COL_OBJECT_TYPES_COUNT*COL_OBJECT_TYPES_COUNT);
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_aabb_aabb;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_aabb_obb;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_aabb_sphere;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_aabb_triangle;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_aabb_point;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_obj_polygon;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_AABB_BOX][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_obb_aabb;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_obb_obb;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_obb_sphere;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_obb_triangle;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_obb_point;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_obj_polygon;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_OBB_BOX][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_sphere_aabb;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_sphere_obb;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_sphere_sphere;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_sphere_triangle;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_sphere_point;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_obj_polygon;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_SPHERE][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_triangle_aabb;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_triangle_obb;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_triangle_sphere;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_triangle_triangle;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_triangle_point;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_obj_polygon;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_TRIANGLE][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_point_aabb;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_point_obb;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_point_sphere;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_point_triangle;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_point_point;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_obj_polygon;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_POINT][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_polygon_obj;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_polygon_obj;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_polygon_obj;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_polygon_obj;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_polygon_obj;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_polygon_polygon;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_obj_shape_group;
        m_collide_bool_function[OBJ_TYPE_POLYGON][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_shape_group_shape_group;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_shape_group_obj;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_shape_group_shape_group;
        m_collide_bool_function[OBJ_TYPE_SHAPEGROUP][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_obj_tvobst;

        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_AABB_BOX] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_OBB_BOX] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_SPHERE] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_TRIANGLE] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_POINT] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_POLYGON] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_SHAPEGROUP] =
solvers::solverPrimitive::collide_tvobst_obj;
        m_collide_bool_function[OBJ_TYPE_TVOBSTACLE][OBJ_TYPE_TVOBSTACLE] =
solvers::solverPrimitive::collide_tvobst_tvobst;
}
*/
}  // namespace solvers
}  // namespace collision
