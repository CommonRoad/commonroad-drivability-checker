#ifndef RECTANGLE_AABB_H_
#define RECTANGLE_AABB_H_

#include <Eigen/Dense>
#include <iostream>

#include <vector>

#include "collision/narrowphase/shape.h"

#include "collision/line_segment.h"

namespace collision {
/*!
  \brief Axis-aligned rectangle

*/
class RectangleAABB : public Shape {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RectangleAABB(double _rx, double _ry,
                const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0))
      : Shape(_center), r_(_rx, _ry) {
    min_ = center_ - r_;
    max_ = center_ + r_;

    segments_.push_back(LineSegment(min_, Eigen::Vector2d(min_.x(), max_.y())));

    segments_.push_back(LineSegment(Eigen::Vector2d(min_.x(), max_.y()), max_));

    segments_.push_back(LineSegment(max_, Eigen::Vector2d(max_.x(), min_.y())));

    segments_.push_back(LineSegment(Eigen::Vector2d(max_.x(), min_.y()), min_));
  }

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const override;

  ~RectangleAABB() {}

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif
  RectangleAABB(const RectangleAABB &copy);
  RectangleAABB *clone() const;

  void print(std::ostringstream &stream) const;

  virtual CollisionObjectType getCollisionObjectType() const override {
    return CollisionObjectType::OBJ_TYPE_AABB_BOX;
  }

  ShapeType type() const;

  Eigen::Vector2d r() const;
  double r(int i) const;
  double r_x() const;
  double r_y() const;
  Eigen::Vector2d min() const;
  Eigen::Vector2d max() const;

  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
      const override;

  void set_center(const Eigen::Vector2d &_center);
  void set_r(const Eigen::Vector2d &_r);
  void set_r_x(double _r_x);
  void set_r_y(double _r_y);
  void set_all(double r_x, double r_y, double center_x, double center_y);

  //
  // @brief Computes the square distance between a point and
  //        the rectangle boundary. From:
  //        C. Ericson, Real-Time Collision Detection, pp. 131, 2004
  // @param p Point
  //
  double squareDisToPoint(const Eigen::Vector2d &p) const;

  std::vector<LineSegment> segments(void) const { return segments_; };

 private:
  // Center-radius representation
  //  ----------- max
  //  |    |ry  |
  //  |   c|____|
  //  |     ry  |
  //  |         |
  //  -----------
  // min
  //
  using Shape::center_;
  using Shape::radius_;

  //! Positive halfwidth extents of OBB along each axis (rx, ry)
  Eigen::Vector2d r_;

  //! Min-Max Points on rectangle
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;

  std::vector<LineSegment> segments_;

  static constexpr ShapeType type_ = TYPE_RECTANGLE_AABB;
};

}  // namespace collision

#endif
