
#ifndef CPP_COLLISION_INCLUDE_COLLISION_FASTAABB2_H_
#define CPP_COLLISION_INCLUDE_COLLISION_FASTAABB2_H_

#include <cmath>
#include "collision/narrowphase/detail/aabb.h"

namespace collision {
namespace detail {
class AABB_SAT2D {
 public:
  AABB_SAT2D(void) : x(0), y(0), half_width(0), half_height(0) {}

  AABB_SAT2D(double x, double y, double half_width, double half_height) {
    this->x = x;
    this->y = y;

    this->half_width = half_width;
    this->half_height = half_height;
  }

  AABB_SAT2D(const collision::AABB& faabb) {
    x = (faabb.x_max + faabb.x_min) / 2;
    y = (faabb.y_max + faabb.y_min) / 2;
    half_width = (faabb.x_max - faabb.x_min) / 2;
    half_height = (faabb.y_max - faabb.y_min) / 2;
  }

  inline bool collides(
      const AABB_SAT2D& other)  // if aabbs touch, they might not collide
  {
    return (fabs(x - other.x) < (half_width + other.half_width)) &&
           (fabs(y - other.y) < (half_height + other.half_height));
  }

  inline bool collides_eps(const AABB_SAT2D& other, double eps) {
    return (fabs(x - other.x) < (half_width + other.half_width) + eps) &&
           (fabs(y - other.y) < (half_height + other.half_height) + eps);
  }
  double x;
  double y;
  double half_width;
  double half_height;
};
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_FASTAABB_H_ */
