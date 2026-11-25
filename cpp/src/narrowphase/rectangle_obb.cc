#include "collision/narrowphase/rectangle_obb.h"

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/raytrace_utils.h"

namespace collision {

/*!
 \brief creates FCL collision geometry for a RectangleOBB. This is a FCL library internal representation used for collision checking.
*/

fcl::CollisionGeometry<FCL_PRECISION> *RectangleOBB::createFCLCollisionGeometry(
    void) const {
  return new fcl::Box<FCL_PRECISION>(r_x() * 2, r_y() * 2, FCL_HEIGHT);
}

/*!
 \brief creates FCL collision object for a RectangleOBB. This is a FCL library internal representation used for collision checking.
 \param[in] col_geom - corresponding FCL collision geometry
*/

fcl::CollisionObject<FCL_PRECISION> *RectangleOBB::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom, collision::FCLTransform::fcl_get_3d_rotation_translation(
                    this->center(), this->local_x_axis()));
}

/*!
 \brief A helper function that is called from the rayTracePrimitive function.
 Given the query line segment [point1, point2], it outputs the part(s) of the line segment that intersect with the RectangleOBB.
 \param[in] point1 - start of the query line segment
 \param[in] point2 - end of the query line segment
 \param[out] intersect - vector to which the output line segments are to be appended
*/

bool RectangleOBB::rayTrace(const Eigen::Vector2d &point1,
                            const Eigen::Vector2d &point2,
                            std::vector<LineSegment> &intersect) const {
  LineSegment ray_segment(point1, point2);

  std::vector<Eigen::Vector2d> inters1;
  bool res = false;
  for (auto &segm : segments_) {
    res = segm.intersect(ray_segment, inters1) || res;
  }
  collision::raytrace::rayTracePostprocess(point1, point2, inters1, intersect,
                                           this);

  return res;
}

/*!
 \brief Clones the RectangleOBB
*/

RectangleOBB *RectangleOBB::clone() const { return new RectangleOBB(*this); }

/*!
 \brief Returns the type of the Shape
*/

ShapeType RectangleOBB::type() const { return type_; }

/*!
 \brief Prints out important information about the RectangleOBB
 \param[out] stream - output stringstream to print the information to
*/

void RectangleOBB::print(std::ostringstream &stream) const {
  stream << "OBB Rectangle: center: (" << center_x() << "/" << center_y()
         << "), r: (" << r_(0) << "|" << r_(1) << ") "
         << "Local coordinate axes: (" << local_axes_(0, 0) << ","
         << local_axes_(1, 0) << "), (" << local_axes_(0, 1) << ","
         << local_axes_(1, 1) << ")" << std::endl;
}

/*!
 \brief getter for local_axes_
*/

Eigen::Matrix2d RectangleOBB::local_axes() const { return local_axes_; }

/*!
 \brief getter for the first column vector (local x-axis) of the local_axes_ matrix
*/

Eigen::Vector2d RectangleOBB::local_x_axis() const {
  return local_axes_.col(0);
}

/*!
 \brief getter for the second column vector (local y-axis) of the local_axes_ matrix
*/

Eigen::Vector2d RectangleOBB::local_y_axis() const {
  return local_axes_.col(1);
}

/*!
 \brief getter for r_ (radius vector). The radius is the the OBB box (half-width, half-height) along the local x- and y- axis respectively.
*/

Eigen::Vector2d RectangleOBB::r() const { return r_; }

/*!
 \brief getter for a component of r_ (radius vector). The radius is the the OBB box (half-width, half-height) along the local x- and y- axis respectively.
 \param[in] i - index of the component
*/

double RectangleOBB::r(int i) const {
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
 \brief getter for the x component of r_ (radius vector). It is the RectangleOBB half-width along the local x-axis.
*/

double RectangleOBB::r_x() const { return r_(0); }


/*!
 \brief getter for the y component of r_ (radius vector). It is the RectangleOBB half-height along the local y-axis.
*/

double RectangleOBB::r_y() const { return r_(1); }

/*!
 \brief setter for the first column vector (local x-axis) of the local_axes_ matrix
*/

void RectangleOBB::set_local_x_axis(const Eigen::Vector2d &x_axis) {
  local_axes_.col(0) = x_axis;
  invalidateCollisionEntityCache();
  //is_fastAABB_cached_ = false;
  is_orientation_cached_ = false;
  segments_.clear();
  set_up_segments(); // also recomputes fastAABB
}

/*!
 \brief setter for the second column vector (local y-axis) of the local_axes_ matrix
*/

void RectangleOBB::set_local_y_axis(const Eigen::Vector2d &y_axis) {
  local_axes_.col(1) = y_axis;
  invalidateCollisionEntityCache();
  //is_fastAABB_cached_ = false;
  is_orientation_cached_ = false;
  segments_.clear();
  set_up_segments(); // also recomputes fastAABB
}

/*!
 \brief setter for the center of the RectangleOBB.
*/

void RectangleOBB::set_center(const Eigen::Vector2d &_center) {
  center_ = _center;
  invalidateCollisionEntityCache();
  //is_fastAABB_cached_ = false;
  segments_.clear();
  set_up_segments();
}

/*!
 \brief setter for the x component of r_ (radius vector). It is the RectangleOBB half-width along the local x-axis.
*/

void RectangleOBB::set_r_x(double _r_x) {
  r_(0) = _r_x;
  invalidateCollisionEntityCache();
  //is_fastAABB_cached_ = false;
  segments_.clear();
  set_up_segments(); // also recomputes fastAABB
}

/*!
 \brief setter for the y component of r_ (radius vector). It is the RectangleOBB half-height along the local y-axis.
*/

void RectangleOBB::set_r_y(double _r_y) {
  r_(1) = _r_y;
  invalidateCollisionEntityCache();
  //is_fastAABB_cached_ = false;
  segments_.clear();
  set_up_segments();
}

/*!
 \brief returns the orientation (CCW angle in radians between global x axis and the local x axis) of the RectangleOBB
*/

double RectangleOBB::orientation() const {
  if (is_orientation_cached_)
    return cached_orientation_;
  else
    compute_orientation();
  return cached_orientation_;
}

/*!
 \brief computes the orientation (CCW angle in radians between global x axis and the local x axis) of the RectangleOBB
*/

void RectangleOBB::compute_orientation() const {
  Eigen::Matrix2d temp_matrix;
  temp_matrix.col(0) = local_x_axis();
  temp_matrix.col(1) = Eigen::Vector2d(1, 0);
  double dot = temp_matrix.col(0).dot(temp_matrix.col(1));
  double det = temp_matrix.determinant();
  cached_orientation_ =
      -1 * std::atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
  is_orientation_cached_ = true;
}

/*!
 \brief Returns square of the shortest distance between the RectangleOBB and a given point.
 \param[in] p - input point
*/

double RectangleOBB::squareDisToPoint(const Eigen::Vector2d &p) const {
  double sq_dis = 0.0;
  //! Project translation vector t in OBB's local coordinate system
  Eigen::Vector2d t = local_axes_.transpose() * (p - center_);

  for (int i = 0; i < 2; i++) {
    if (t(i) < -r_(i)) {
      sq_dis += pow(t(i) + r_(i), 2);
    } else if (t(i) > r_(i)) {
      sq_dis += pow(t(i) - r_(i), 2);
    }
  }
  return sq_dis;
}

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::RectangleOBB &);
}

/*!
 \brief Exports the RectangleOBB into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *RectangleOBB::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
