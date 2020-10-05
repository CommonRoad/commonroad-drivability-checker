#include "collision/narrowphase/rectangle_obb.h"

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/raytrace_utils.h"

namespace collision {

fcl::CollisionGeometry<FCL_PRECISION> *RectangleOBB::createFCLCollisionGeometry(
    void) const {
  return new fcl::Box<FCL_PRECISION>(r_x() * 2, r_y() * 2, FCL_HEIGHT);
}
fcl::CollisionObject<FCL_PRECISION> *RectangleOBB::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom, collision::FCLTransform::fcl_get_3d_rotation_translation(
                    this->center(), this->local_x_axis()));
}

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

RectangleOBB *RectangleOBB::clone() const { return new RectangleOBB(*this); }

ShapeType RectangleOBB::type() const { return type_; }

void RectangleOBB::print(std::ostringstream &stream) const {
  stream << "OBB Rectangle: center: (" << center_x() << "/" << center_y()
         << "), r: (" << r_(0) << "|" << r_(1) << ") "
         << "Local coordinate axes: (" << local_axes_(0, 0) << ","
         << local_axes_(1, 0) << "), (" << local_axes_(0, 1) << ","
         << local_axes_(1, 1) << ")" << std::endl;
}

Eigen::Matrix2d RectangleOBB::local_axes() const { return local_axes_; }

Eigen::Vector2d RectangleOBB::local_x_axis() const {
  return local_axes_.col(0);
}

Eigen::Vector2d RectangleOBB::local_y_axis() const {
  return local_axes_.col(1);
}

Eigen::Vector2d RectangleOBB::r() const { return r_; }

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

double RectangleOBB::r_x() const { return r_(0); }

double RectangleOBB::r_y() const { return r_(1); }

void RectangleOBB::set_local_x_axis(const Eigen::Vector2d &x_axis) {
  local_axes_.col(0) = x_axis;
  invalidateCollisionEntityCache();
  is_fastAABB_cached_ = false;
  is_orientation_cached_ = false;
}

void RectangleOBB::set_local_y_axis(const Eigen::Vector2d &y_axis) {
  local_axes_.col(1) = y_axis;
  invalidateCollisionEntityCache();
  is_fastAABB_cached_ = false;
  is_orientation_cached_ = false;
}

void RectangleOBB::set_r_x(double _r_x) {
  r_(0) = _r_x;
  invalidateCollisionEntityCache();
  is_fastAABB_cached_ = false;
}

void RectangleOBB::set_r_y(double _r_y) {
  r_(1) = _r_y;
  invalidateCollisionEntityCache();
  is_fastAABB_cached_ = false;
}

double RectangleOBB::orientation() const {
  if (is_orientation_cached_)
    return cached_orientation_;
  else
    compute_orientation();
  return cached_orientation_;
}

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

serialize::ICollisionObjectExport *RectangleOBB::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
