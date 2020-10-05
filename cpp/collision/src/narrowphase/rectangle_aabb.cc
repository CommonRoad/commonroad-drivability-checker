#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/raytrace_utils.h"

#include "collision/narrowphase/rectangle_aabb.h"

namespace collision {

fcl::CollisionGeometry<FCL_PRECISION>
    *RectangleAABB::createFCLCollisionGeometry(void) const {
  return new fcl::Box<FCL_PRECISION>(r_x() * 2, r_y() * 2, FCL_HEIGHT);
}
fcl::CollisionObject<FCL_PRECISION> *RectangleAABB::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom,
      collision::solvers::solverFCL::FCLTransform::fcl_get_3d_translation(
          this->center()));
}

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

RectangleAABB *RectangleAABB::clone() const { return new RectangleAABB(*this); }

RectangleAABB::RectangleAABB(const RectangleAABB &copy) : Shape(copy) {
  center_ = copy.center();  // not needed? It is already set in Shape(copy)
  radius_ = copy.radius();
  r_ = copy.r();
  min_ = copy.min();
  max_ = copy.max();
  segments_ = copy.segments();
}

ShapeType RectangleAABB::type() const { return type_; }

void RectangleAABB::print(std::ostringstream &stream) const {
  stream << "AABB Rectangle: center: (" << center_x() << "/" << center_y()
         << ") r: (" << r_(0) << "|" << r_(1) << ") "
         << "min: (" << min_(0) << "|" << min_(1) << ") "
         << "max: (" << max_(0) << "|" << max_(1) << ")" << std::endl;
}

Eigen::Vector2d RectangleAABB::min() const { return min_; }

Eigen::Vector2d RectangleAABB::max() const { return max_; }

Eigen::Vector2d RectangleAABB::r() const { return r_; }

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

void RectangleAABB::set_r(const Eigen::Vector2d &_r) {
  r_ = _r;
  min_ = center_ - r_;
  max_ = center_ + r_;
  invalidateCollisionEntityCache();
}

double RectangleAABB::r_x() const { return r_(0); }

double RectangleAABB::r_y() const { return r_(1); }

void RectangleAABB::set_center(const Eigen::Vector2d &_center) {
  center_ = _center;
  min_ = center_ - r_;
  max_ = center_ + r_;
  invalidateCollisionEntityCache();
}

void RectangleAABB::set_r_x(double _r_x) {
  r_(0) = _r_x;
  min_(0) = center_(0) - r_(0);
  max_(0) = center_(0) + r_(0);
  invalidateCollisionEntityCache();
}

void RectangleAABB::set_r_y(double _r_y) {
  r_(1) = _r_y;
  min_(1) = center_(1) - r_(1);
  max_(1) = center_(1) + r_(1);
  invalidateCollisionEntityCache();
}

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
}

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

serialize::ICollisionObjectExport *RectangleAABB::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
