#include "collision/solvers/collision_queries.h"
#include "collision/solvers/collision_solvers.h"

namespace collision {
namespace detail {
// default solver
template <typename T>
inline std::size_t collide_binary_helper(const CollisionObject &obj1,
                                         const CollisionObject &obj2,
                                         CollisionResult &res,
                                         const CollisionRequest &req) {
  static solvers::DefaultSolver solver;
  static solvers::CollisionFunctionMatrix matr_default(&solver);
  auto func = matr_default.getSolverBoolFunction(obj1.getCollisionObjectType(),
                                                 obj2.getCollisionObjectType());
  return func(obj1, obj2, res, req);
}

template <typename solvers::DefaultSolver *>
inline std::size_t collide_binary_helper(const CollisionObject &obj1,
                                         const CollisionObject &obj2,
                                         CollisionResult &res,
                                         const CollisionRequest &req);

// solverFCL solver
template <>
inline std::size_t collide_binary_helper<typename solvers::FCLSolver *>(
    const CollisionObject &obj1, const CollisionObject &obj2,
    CollisionResult &res, const CollisionRequest &req) {
  static solvers::FCLSolver solver;
  static solvers::CollisionFunctionMatrix matr_default(&solver);
  auto func = matr_default.getSolverBoolFunction(obj1.getCollisionObjectType(),
                                                 obj2.getCollisionObjectType());
  return func(obj1, obj2, res, req);
}

// primitive solver
/*
 template<> inline std::size_t collide_binary_helper<typename
 solvers::PRIMITIVE_SOLVER>(const CollisionObject& obj1, const CollisionObject&
 obj2, CollisionResult& res, const CollisionRequest& req)
 {
 static solvers::PRIMITIVE_SOLVER solver;
 static solvers::CollisionFunctionMatrix matr_default(solver);
 auto func = matr_default.getSolverBoolFunction(obj1.getCollisionObjectType(),
 obj2.getCollisionObjectType()); return func(obj1, obj2, res, req);
 }
 */
}  // namespace detail

std::size_t collide_binary(const CollisionObject &obj1,
                           const CollisionObject &obj2, CollisionResult &res,
                           const CollisionRequest &req) {
  switch (req.col_solver_type) {
    case COL_DEFAULT:
      return detail::collide_binary_helper<solvers::DefaultSolver *>(obj1, obj2,
                                                                     res, req);
      break;
    case COL_FCL:
      return detail::collide_binary_helper<solvers::FCLSolver *>(obj1, obj2,
                                                                 res, req);
      break;
    default:
      return -1;
      break;
      // case COL_PRIMITIVE:
      //	return
      // detail::collide_binary_helper<solvers::PRIMITIVE_SOLVER>(obj1, obj2,
      // res, req); 	break;
  }
}

}  // namespace collision
