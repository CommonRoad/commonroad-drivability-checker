#ifndef CPP_COLLISION_INCLUDE_COLLISION_FASTAABB_H_
#define CPP_COLLISION_INCLUDE_COLLISION_FASTAABB_H_

#include "collision/narrowphase/rectangle_aabb.h"

namespace collision {
class AABB {
 public:
  double x_min;
  double x_max;
  double y_min;
  double y_max;

  AABB(void) : x_min(0), x_max(0), y_min(0), y_max(0) {}

  AABB(double x_min, double x_max, double y_min, double y_max) {
    this->x_min = x_min;
    this->x_max = x_max;

    this->y_min = y_min;
    this->y_max = y_max;
  }

  AABB(const RectangleAABB& aabb_rect) {
    x_min = aabb_rect.min()[0];
    y_min = aabb_rect.min()[1];
    x_max = aabb_rect.max()[0];
    y_max = aabb_rect.max()[1];
  }

  bool collides(const AABB& other);

  void swapAxes(void) {
    std::swap(x_min, y_min);
    std::swap(x_max, y_max);
  }
};
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_FASTAABB_H_ */
