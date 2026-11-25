#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/raytrace_utils.h"

#include "collision/narrowphase/rectangle_aabb.h"

namespace collision {

/*!
 \brief creates FCL collision geometry for a RectangleAABB. This is a FCL library internal representation used for collision checking.
*/

fcl::CollisionGeometry<FCL_PRECISION>
    *RectangleAABB::createFCLCollisionGeometry(void) const {
  return new fcl::Box<FCL_PRECISION>(r_x() * 2, r_y() * 2, FCL_HEIGHT);
}

/*!
 \brief creates FCL collision object for a RectangleAABB. This is a FCL library internal representation used for collision checking.
 \param[in] col_geom - corresponding FCL collision geometry
*/

fcl::CollisionObject<FCL_PRECISION> *RectangleAABB::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom,
      collision::solvers::solverFCL::FCLTransform::fcl_get_3d_translation(
          this->center()));
}


/*!
 \brief A helper function that is called from the rayTracePrimitive function.
 Given the query line segment [point1, point2], it outputs the part(s) of the line segment that intersect with the RectangleAABB.
 \param[in] point1 - start of the query line segment
 \param[in] point2 - end of the query line segment
 \param[out] intersect - vector to which the output line segments are to be appended
*/

bool RectangleAABB::rayTrace(const Eigen::Vector2d &point1,
                             const Eigen::Vector2d &point2,
                             std::vector<LineSegment> &intersect) const {
  LineSegment obj_segment(point1, point2);

  std::vector<Eigen::Vector2d> inters1;
  bool res = false;
  for (auto &segm : segments_) {
    res = segm.intersect(obj_segment, inters1) || res;
  }
  collision::raytrace::rayTracePostprocess(point1, point2, inters1, intersect,
                                           this);

  return res;
}

/*!
 \brief Clones the RectangleAABB
*/

RectangleAABB *RectangleAABB::clone() const { return new RectangleAABB(*this); }

/*!
 \brief Copy constructor for a RectangleAABB
*/

RectangleAABB::RectangleAABB(const RectangleAABB &copy) : Shape(copy) {
  r_ = copy.r();
  min_ = copy.min();
  max_ = copy.max();
  segments_ = copy.segments();
}

/*!
 \brief Returns the type of the Shape
*/

ShapeType RectangleAABB::type() const { return type_; }

/*!
 \brief Prints out important information about the RectangleAABB
 \param[out] stream - output stringstream to print the information to
*/

void RectangleAABB::print(std::ostringstream &stream) const {
  stream << "AABB Rectangle: center: (" << center_x() << "/" << center_y()
         << ") r: (" << r_(0) << "|" << r_(1) << ") "
         << "min: (" << min_(0) << "|" << min_(1) << ") "
         << "max: (" << max_(0) << "|" << max_(1) << ")" << std::endl;
}

/*!
 \brief getter for min_ 2D point
*/

Eigen::Vector2d RectangleAABB::min() const { return min_; }

/*!
 \brief getter for max_ 2D point
*/

Eigen::Vector2d RectangleAABB::max() const { return max_; }

/*!
 \brief getter for r_ (radius). The radius is a 2D vector (half of the rectangle width, half of the rectangle height).
*/

Eigen::Vector2d RectangleAABB::r() const { return r_; }

/*!
 \brief getter for individual components of r_ (radius).
 The radius is a 2D vector (half of the rectangle width, half of the rectangle height).
 \param[in] i - the index of the radius vector component
*/

double RectangleAABB::r(int i) const {
  switch (i) {
    case 0:
      return r_(0);
    case 1:
      return r_(1);
    default:
      throw "Rectangle_OBB: Not a valid index for r";
  }
}

/*!
 \brief setter for r_ (radius). Also recomputes the min_ and max_ points.
 The radius is a 2D vector (half of the rectangle width, half of the rectangle height).
 \param[in] _r - new radius vector
*/

void RectangleAABB::set_r(const Eigen::Vector2d &_r) {
  r_ = _r;
  min_ = center_ - r_;
  max_ = center_ + r_;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_up_segments();
}

/*!
 \brief returns half-width of the axis-aligned rectangle
*/

double RectangleAABB::r_x() const { return r_(0); }

/*!
 \brief returns half-height of the axis-aligned rectangle
*/

double RectangleAABB::r_y() const { return r_(1); }

/*!
 \brief setter for center_ (center point). Also recomputes the min_ and max_ points.
 \param[in] _center - new center
*/

void RectangleAABB::set_center(const Eigen::Vector2d &_center) {
  center_ = _center;
  min_ = center_ - r_;
  max_ = center_ + r_;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_up_segments();
}

/*!
 \brief sets the half-width of the axis-aligned rectangle. Also recomputes the min_ and max_ points.
 \param[in] _r_x - new half-wdth
*/

void RectangleAABB::set_r_x(double _r_x) {
  r_(0) = _r_x;
  min_(0) = center_(0) - r_(0);
  max_(0) = center_(0) + r_(0);
  invalidateCollisionEntityCache();
  segments_.clear();
  set_up_segments();
}

/*!
 \brief sets the half-height of the axis-aligned rectangle. Also recomputes the min_ and max_ points.
 \param[in] _r_y - new half-height
*/

void RectangleAABB::set_r_y(double _r_y) {
  r_(1) = _r_y;
  min_(1) = center_(1) - r_(1);
  max_(1) = center_(1) + r_(1);
  invalidateCollisionEntityCache();
  segments_.clear();
  set_up_segments();
}

/*!
 \brief sets the radius and the center of the axis-aligned rectangle. Also recomputes the min_ and max_ points.
 The radius is a 2D vector (half of the rectangle width, half of the rectangle height).
 \param[in] _r_x - new half-width
 \param[in] _r_y - new half-height
 \param[in] center_x - new center x coordinate
 \param[in] center_y - new center y coordinate
*/

void RectangleAABB::set_all(double r_x, double r_y, double center_x,
                            double center_y) {
  center_(0) = center_x;
  r_(0) = r_x;
  min_(0) = center_x - r_x;
  max_(0) = center_x + r_x;

  center_(1) = center_y;
  r_(1) = r_y;
  min_(1) = center_y - r_y;
  max_(1) = center_y + r_y;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_up_segments();
}

/*!
 \brief Returns square of the shortest distance between the RectangleAABB and a given point.
 \param[in] p - input point
*/

double RectangleAABB::squareDisToPoint(const Eigen::Vector2d &p) const {
  double sq_dis = 0.0;
  for (int i = 0; i < 2; i++) {
    if (p(i) < min_(i)) {
      sq_dis += pow(min_(i) - p(i), 2);
    } else if (p(i) > max_(i)) {
      sq_dis += pow(p(i) - max_(i), 2);
    }
  }
  return sq_dis;
}

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::RectangleAABB &);
}

/*!
 \brief Exports the RectangleAABB into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *RectangleAABB::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
