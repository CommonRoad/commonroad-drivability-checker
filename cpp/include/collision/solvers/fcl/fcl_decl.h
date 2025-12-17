#ifndef COLLISION_FCL_FCL_DECL_H_
#define COLLISION_FCL_FCL_DECL_H_

namespace collision {
namespace solvers {
namespace solverFCL {
typedef double FCL_PRECISION;

#define FCL_HEIGHT (0.1)
#define FCL_SOLVER_TYPE (fcl::GST_LIBCCD)

}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision

#endif /* COLLISION_FCL_FCL_DECL_H_ */
