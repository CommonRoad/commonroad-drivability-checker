#ifndef COLLISION_FCL_FCL_DECL_H_
#define COLLISION_FCL_FCL_DECL_H_

namespace collision {
namespace solvers {
namespace solverFCL {
typedef double FCL_PRECISION;
#define DEFAULT_DISTANCE_TOLERANCE 1e-6

#define FCL_HEIGHT (0.1)
#define FCL_SOLVER_TYPE (fcl::GST_LIBCCD)

enum FCL_TOLERANCE_CHECK_TYPE {
  TOLERANCE_CHECK_NARROWPHASE = 0,
  TOLERANCE_CHECK_BB = 1
};
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision

#endif /* COLLISION_FCL_FCL_DECL_H_ */
