#ifndef RECTANGLE_OBB_H_
#define RECTANGLE_OBB_H_

#include <exception>
#include <iostream>

#include "collision/narrowphase/detail/obb.h"
#include "collision/narrowphase/shape.h"

#include "collision/line_segment.h"

#include "detail/aabb.h"

namespace collision {
/*!
  \brief Oriented rectangle

*/
class RectangleOBB : public Shape {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void set_up_segments(void) {
    // must be clockwise to be able to correctly construct BoostPolygon

    Eigen::Vector2d _v1 =
        center() - r_x() * local_x_axis() + r_y() * local_y_axis();
    Eigen::Vector2d _v2 =
        center() + r_x() * local_x_axis() + r_y() * local_y_axis();
    Eigen::Vector2d _v3 =
        center() + r_x() * local_x_axis() - r_y() * local_y_axis();
    Eigen::Vector2d _v4 =
        center() - r_x() * local_x_axis() - r_y() * local_y_axis();

    compute_fastAABB(_v1, _v2, _v3, _v4);

    segments_.push_back(LineSegment(_v1, _v2));
    segments_.push_back(LineSegment(_v2, _v3));
    segments_.push_back(LineSegment(_v3, _v4));
    segments_.push_back(LineSegment(_v4, _v1));
  }

  inline void compute_fastAABB(const Eigen::Vector2d &_v1,
                               const Eigen::Vector2d &_v2,
                               const Eigen::Vector2d &_v3,
                               const Eigen::Vector2d &_v4) const {
    double min_x = std::min(_v1(0), _v2(0));
    double tmp = std::min(_v3(0), _v4(0));
    fast_aabb_.x_min = std::min(min_x, tmp);

    double min_y = std::min(_v1(1), _v2(1));
    tmp = std::min(_v3(1), _v4(1));
    fast_aabb_.y_min = std::min(min_y, tmp);

    double max_x = std::max(_v1(0), _v2(0));
    tmp = std::max(_v3(0), _v4(0));
    fast_aabb_.x_max = std::max(max_x, tmp);

    double max_y = std::max(_v1(1), _v2(1));
    tmp = std::max(_v3(1), _v4(1));
    fast_aabb_.y_max = std::max(max_y, tmp);

    is_fastAABB_cached_ = true;
  }

  inline void compute_fastAABB(void) const {
    double min_x = std::min(segments_[0].point1().x, segments_[1].point1().x);
    double tmp = std::min(segments_[2].point1().x, segments_[3].point1().x);
    fast_aabb_.x_min = std::min(min_x, tmp);

    double min_y = std::min(segments_[0].point1().y, segments_[1].point1().y);
    tmp = std::min(segments_[2].point1().y, segments_[3].point1().y);
    fast_aabb_.y_min = std::min(min_y, tmp);

    double max_x = std::max(segments_[0].point1().x, segments_[1].point1().x);
    tmp = std::max(segments_[2].point1().x, segments_[3].point1().x);
    fast_aabb_.x_max = std::max(max_x, tmp);

    double max_y = std::max(segments_[0].point1().y, segments_[1].point1().y);
    tmp = std::max(segments_[2].point1().y, segments_[3].point1().y);
    fast_aabb_.y_max = std::max(max_y, tmp);

    is_fastAABB_cached_ = true;
  }

  RectangleOBB(const detail::OBB &obb)
      : RectangleOBB(obb.r_x(), obb.r_y(), obb.local_axes(), obb.center()){};

  RectangleOBB(double _r_x, double _r_y, Eigen::Matrix2d _local_axes,
               const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0))
      : Shape(_center),
        local_axes_(_local_axes),
        r_(_r_x, _r_y),
        is_fastAABB_cached_(false),
        cached_orientation_(0),
        is_orientation_cached_(false) {
    set_up_segments();
  }

  RectangleOBB(double _r_x, double _r_y, double angle,
               const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0))
      : Shape(_center),
        r_(_r_x, _r_y),
        is_fastAABB_cached_(false),
        cached_orientation_(angle),
        is_orientation_cached_(true) {
    local_axes_ << cos(angle), -sin(angle), sin(angle), cos(angle);

    set_up_segments();
  }

  RectangleOBB(const RectangleOBB &copy)
      : Shape(copy), is_fastAABB_cached_(false) {
    center_ = copy.center();
    radius_ = copy.radius();
    local_axes_ = copy.local_axes();
    r_ = copy.r();
    segments_ = copy.segments();
    cached_orientation_ = copy.cached_orientation_;
    is_orientation_cached_ = copy.is_orientation_cached_;
  }

  double orientation() const;

  inline AABB getAABB_fast(void) const {
    if (!is_fastAABB_cached_) {
      compute_fastAABB();
    }
    return fast_aabb_;
  }

  virtual CollisionObjectType getCollisionObjectType() const {
    return CollisionObjectType::OBJ_TYPE_OBB_BOX;
  }

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const override;

  virtual ~RectangleOBB() {}

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif

  virtual RectangleOBB *clone() const;

  void print(std::ostringstream &stream) const;

  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &)
      const override;

  ShapeType type() const;

  Eigen::Matrix2d local_axes() const;
  Eigen::Vector2d local_x_axis() const;
  Eigen::Vector2d local_y_axis() const;
  Eigen::Vector2d r() const;
  double r(int i) const;
  double r_x() const;
  double r_y() const;

  void set_local_x_axis(const Eigen::Vector2d &x_axis);
  void set_local_y_axis(const Eigen::Vector2d &y_axis);
  void set_r_x(double _r_x);
  void set_r_y(double _r_y);

  //
  // @brief Computes the square distance between a point and
  //        the rectangle boundary. From:
  //        C. Ericson, Real-Time Collision Detection, pp. 134, 2004
  // @param p Point
  //
  double squareDisToPoint(const Eigen::Vector2d &p) const;

  std::vector<LineSegment> segments(void) const { return segments_; };

 private:
  using Shape::center_;
  using Shape::radius_;

  // Local x- and y-axis (column vectors)
  //               | x_1  x_2 |
  //  local_axis = |          |
  //               | y_1  y_2 |
  //
  Eigen::Matrix2d local_axes_;

  //! Positive halfwidth extents of OBB along each axis
  Eigen::Vector2d r_;

  std::vector<LineSegment> segments_;

  static constexpr ShapeType type_ = TYPE_RECTANGLE_OBB;

  mutable bool is_fastAABB_cached_;

  mutable AABB fast_aabb_;

  mutable double cached_orientation_;
  mutable bool is_orientation_cached_;

  void compute_orientation(void) const;
};

}  // namespace collision

#endif
