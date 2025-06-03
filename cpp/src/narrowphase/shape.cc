#include "collision/narrowphase/shape.h"

namespace collision {

Shape::~Shape() {}

Shape::Shape(const Shape &copy) : center_(copy.center()) {}

// void Shape::print(std::ostringstream &stream) const {}

CollisionObjectConstPtr Shape::timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return shared_ptr_this;
}

Eigen::Vector2d Shape::center() const { return center_; }

double Shape::center_x() const { return center_(0); }

double Shape::center_y() const { return center_(1); }

void Shape::set_center(const Eigen::Vector2d &_center) {
  center_ = _center;
  invalidateCollisionEntityCache();
}

double Shape::radius() const { return radius_; }

}  // namespace collision
