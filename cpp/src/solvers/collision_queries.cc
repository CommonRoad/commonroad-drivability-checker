#include "collision/solvers/collision_queries.h"
#include "collision/solvers/collision_solvers.h"

namespace collision {
namespace detail {

/*!
 \brief Helper function for the default collision solver
 It dispatches the collision query to the necessary collision detection function of the solver.
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

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

/*!
 \brief Helper function for using the FCL library collision solver
 It dispatches the collision query to the necessary collision detection function of the solver.
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

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

/*!
 \brief Helper function for using the in-built 2D collision solver.
 It dispatches the collision query to the necessary collision detection function of the solver.
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

template<> inline std::size_t collide_binary_helper<
		typename solvers::PrimitiveSolver*>(const CollisionObject &obj1,
		const CollisionObject &obj2, CollisionResult &res,
		const CollisionRequest &req) {
	static solvers::PrimitiveSolver solver;
	static solvers::CollisionFunctionMatrix matr_default(&solver);
	auto func = matr_default.getSolverBoolFunction(
			obj1.getCollisionObjectType(), obj2.getCollisionObjectType());
	return func(obj1, obj2, res, req);
}

}  // namespace detail

/*!
 \brief Checks whether two collision objects collide or not.
 \param[in] obj1 - first collision object
 \param[in] obj2 - second collision object
 \param[out] res - result of the collision checking
 \param[in] req - parameters for collision detection
*/

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
	case COL_PRIMITIVE:
		return detail::collide_binary_helper<solvers::PrimitiveSolver*>(obj1,
				obj2, res, req);
		break;
    default:
      return -1;
      break;
  }
}

}  // namespace collision
