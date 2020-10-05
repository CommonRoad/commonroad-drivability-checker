#include "collision/narrowphase/detail/aabb.h"
#include "collision/solvers/sat2d/aabb_sat2d.h"

namespace collision {

bool AABB::collides(const AABB& other) {
  collision::detail::AABB_SAT2D a(*this);
  collision::detail::AABB_SAT2D b(*this);
  return a.collides(b);
};
}  // namespace collision
