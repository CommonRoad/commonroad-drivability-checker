#include "collision/narrowphase/shape.h"

namespace collision {

/*!
 \brief Shape destructor
*/

Shape::~Shape() {}

/*!
 \brief Copy constructor for Shape
*/

Shape::Shape(const Shape &copy) : center_(copy.center()), radius_(copy.radius()) {}

/*!
 \brief Helper function used in CollisionChecker::timeSlice that returns the static obstacle itself whichever query time step was given
*/

CollisionObjectConstPtr Shape::timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return shared_ptr_this;
}

/*!
 \brief getter for center_
*/

Eigen::Vector2d Shape::center() const { return center_; }

/*!
 \brief getter for center_ x coordinate
*/

double Shape::center_x() const { return center_(0); }

/*!
 \brief getter for center_ y coordinate
*/

double Shape::center_y() const { return center_(1); }

/*!
 \brief Setter for center_. The function may be overridden in order to e.g. recompute cached information whenever the center_ changes.
*/

void Shape::set_center(const Eigen::Vector2d &_center) {
  center_ = _center;
  invalidateCollisionEntityCache();
}

/*!
 \brief getter for Shape radius
*/

double Shape::radius() const { return radius_; }

}  // namespace collision
