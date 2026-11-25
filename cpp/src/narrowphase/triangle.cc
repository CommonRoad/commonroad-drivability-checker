#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include <fcl/geometry/bvh/BVH_model.h>
#include <math.h>
#include "collision/raytrace_utils.h"

#include "collision/narrowphase/triangle.h"

namespace collision {

/*!
 \brief copy constructor for a Triangle
*/

Triangle::Triangle(const Triangle& copy): Shape(copy), v1_(copy.v1_), v2_(copy.v2_), v3_(copy.v3_) {
	segments_ = copy.segments_;
	incircle_radius_ = copy.incircle_radius_;
	incenter_ = copy.incenter_;
	is_valid_ = copy.is_valid_;
}

/*!
 \brief Clones the Triangle
*/

Triangle *Triangle::clone() const { return new Triangle(*this); }

/*!
 \brief A helper function that is called from the rayTracePrimitive function.
 Given the query line segment [point1, point2], it outputs the part(s) of the line segment that intersect with the Triangle.
 \param[in] point1 - start of the query line segment
 \param[in] point2 - end of the query line segment
 \param[out] intersect - vector to which the output line segments are to be appended
*/

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

/*!
 \brief Returns the type of the Shape
*/

ShapeType Triangle::type() const { return type_; }

/*!
 \brief Prints out important information about the Triangle
 \param[out] stream - output stringstream to print the information to
*/

void Triangle::print(std::ostringstream &stream) const {
  stream << "Triangle: \nVertices:"
         << "(" << v1_(0) << "|" << v1_(1) << "), "
         << "(" << v2_(0) << "|" << v2_(1) << "), "
         << "(" << v3_(0) << "|" << v3_(1) << "), "
         << "\ncenter: "
         << "(" << center_x() << "|" << center_y() << "), " << std::endl;
}

/*!
 \brief creates FCL collision geometry for a Triangle. This is a FCL library internal representation used for collision checking.
*/

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

/*!
 \brief creates FCL collision object for a Triangle. This is a FCL library internal representation used for collision checking.
 \param[in] col_geom - corresponding FCL collision geometry
*/

fcl::CollisionObject<FCL_PRECISION> *Triangle::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(col_geom);
}

/*!
 \brief getter for the coordinate vector of Vertex 1
*/

Eigen::Vector2d Triangle::v1() const { return v1_; }

/*!
 \brief getter for the coordinate vector of Vertex 2
*/

Eigen::Vector2d Triangle::v2() const { return v2_; }

/*!
 \brief getter for the coordinate vector of Vertex 3
*/

Eigen::Vector2d Triangle::v3() const { return v3_; }

/*!
 \brief getter for the coordinate vector of the Triangle center_
*/

Eigen::Vector2d Triangle::center() const { return center_; }

/*!
 \brief getter for the coordinate vector of the Triangle incenter_. It is computed in the function compute_incircle_radius_and_center
*/

Eigen::Vector2d Triangle::incenter() const { return incenter_; }

/*!
 \brief getter for the incircle radius of the Triangle. It is computed in the function compute_incircle_radius_and_center
*/

double Triangle::incircle_radius() const { return incircle_radius_; }

/*!
 \brief setter for the coordinate vector of Vertex 1
*/

void Triangle::set_v1(const Eigen::Vector2d &_v1) {
  v1_ = _v1;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_cached_fields();
}

/*!
 \brief setter for the coordinate vector of Vertex 2
*/

void Triangle::set_v2(const Eigen::Vector2d &_v2) {
  v2_ = _v2;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_cached_fields();
}

/*!
 \brief setter for the coordinate vector of Vertex 3
*/

void Triangle::set_v3(const Eigen::Vector2d &_v3) {
  v3_ = _v3;
  invalidateCollisionEntityCache();
  segments_.clear();
  set_cached_fields();
}

/*!
 \brief computes the center of the Triangle based on its vertex coordinates
*/

Eigen::Vector2d Triangle::compute_center() {
  double x = (v1_(0) + v2_(0) + v3_(0)) / 3.0;
  double y = (v1_(1) + v2_(1) + v3_(1)) / 3.0;
  return Eigen::Vector2d(x, y);
}

/*!
 \brief Computes the signed area of the Triangle
*/

double Triangle::compute_signed_area() const {
	double area = 0.0;
	area += (v1_.x() * v2_.y() - v2_.x() * v1_.y());
	area += (v2_.x() * v3_.y() - v3_.x() * v2_.y());
	area += (v3_.x() * v1_.y() - v1_.x() * v3_.y());
	return area / 2.0;
}

/*!
 \brief Checks if the Triangle is valid. If a side is too small or the smallest altitude is too small then the triangle is considered to be invalid.
The function sets the is_valid_ member field. If the is_valid_ field is set to false, all collision functions will return false for such Triangle.
*/

void Triangle::compute_is_valid() {
	double area = fabs(compute_signed_area());

	double side1_sqn = (v2_ - v1_).squaredNorm();
	double side2_sqn = (v3_ - v1_).squaredNorm();
	double side3_sqn = (v3_ - v2_).squaredNorm();

	auto max_side = sqrt(std::max( { side1_sqn, side2_sqn, side3_sqn }));
	auto min_side = sqrt(std::min( { side1_sqn, side2_sqn, side3_sqn }));

	// if a side is too small or the smallest altitude is too small then the triangle is invalid
	if ((min_side < 1e-10) || (area / max_side) < 1e-10) {
		is_valid_ = false;
	} else {
		is_valid_ = true;
	}
}

/*!
 \brief Computes and sets incircle_radius_ and incenter_
*/

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

/*!
 \brief Exports the Triangle into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *Triangle::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
