#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include <fcl/geometry/bvh/BVH_model.h>
#include <math.h>
#include "collision/raytrace_utils.h"

#include "collision/narrowphase/triangle.h"

namespace collision {

Triangle *Triangle::clone() const { return new Triangle(*this); }

bool Triangle::rayTrace(const Eigen::Vector2d &point1,
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

ShapeType Triangle::type() const { return type_; }

void Triangle::print(std::ostringstream &stream) const {
  stream << "Triangle: \nVertices:"
         << "(" << v1_(0) << "|" << v1_(1) << "), "
         << "(" << v2_(0) << "|" << v2_(1) << "), "
         << "(" << v3_(0) << "|" << v3_(1) << "), "
         << "\ncenter: "
         << "(" << center_x() << "|" << center_y() << "), " << std::endl;
}

fcl::CollisionGeometry<FCL_PRECISION> *Triangle::createFCLCollisionGeometry(
    void) const {
  fcl::BVHModel<fcl::AABB<FCL_PRECISION>> *model =
      new fcl::BVHModel<fcl::AABB<FCL_PRECISION>>();
  model->beginModel(1, 3);

  Eigen::Vector2d v2d = v1();
  fcl::Vector3<FCL_PRECISION> v1(v2d[0], v2d[1], 0);
  v2d = v2();
  fcl::Vector3<FCL_PRECISION> v2(v2d[0], v2d[1], 0);
  v2d = v3();
  fcl::Vector3<FCL_PRECISION> v3(v2d[0], v2d[1], 0);

  model->addTriangle(v1, v2, v3);
  model->endModel();
  return model;
}

fcl::CollisionObject<FCL_PRECISION> *Triangle::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(col_geom);
}

Eigen::Vector2d Triangle::v1() const { return v1_; }

Eigen::Vector2d Triangle::v2() const { return v2_; }

Eigen::Vector2d Triangle::v3() const { return v3_; }

Eigen::Vector2d Triangle::center() const { return center_; }

Eigen::Vector2d Triangle::incenter() const { return incenter_; }

double Triangle::incircle_radius() const { return incircle_radius_; }

void Triangle::set_v1(const Eigen::Vector2d &_v1) {
  v1_ = _v1;
  invalidateCollisionEntityCache();
}

void Triangle::set_v2(const Eigen::Vector2d &_v2) {
  v2_ = _v2;
  invalidateCollisionEntityCache();
}

void Triangle::set_v3(const Eigen::Vector2d &_v3) {
  v3_ = _v3;
  invalidateCollisionEntityCache();
}

Eigen::Vector2d Triangle::compute_center() {
  double x = (v1_(0) + v2_(0) + v3_(0)) / 3.0;
  double y = (v1_(1) + v2_(1) + v3_(1)) / 3.0;
  return Eigen::Vector2d(x, y);
}

void Triangle::compute_incircle_radius_and_center() {
  // length of triangle sides
  double a = (v2_ - v1_).norm();
  double b = (v3_ - v2_).norm();
  double c = (v3_ - v1_).norm();
  // half of perimeter
  double p = 0.5 * (a + b + c);
  // using Heron's Formula
  double area = std::sqrt(p * (p - a) * (p - b) * (p - c));
  // incenter
  incenter_ = (b * v1_ + c * v2_ + a * v3_) / (2.0 * p);
  // inradius
  incircle_radius_ = area / p;
}

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::Triangle &);
}

serialize::ICollisionObjectExport *Triangle::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
