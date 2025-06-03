#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <assert.h>
#include <Eigen/Dense>
#include <iostream>

#include "collision/line_segment.h"
#include "collision/narrowphase/shape.h"

namespace collision {

/*!
  \brief Triangle

*/
class Triangle : public Shape {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Triangle(const Eigen::Vector2d &_v1 = Eigen::Vector2d(0, 0),
           const Eigen::Vector2d &_v2 = Eigen::Vector2d(0, 0),
           const Eigen::Vector2d &_v3 = Eigen::Vector2d(0, 0))
      : Shape(Eigen::Vector2d(0, 0)), v1_(_v1), v2_(_v2), v3_(_v3) {
    set_center(compute_center());
    compute_incircle_radius_and_center();
    segments_.push_back(LineSegment(_v1, _v2));
    segments_.push_back(LineSegment(_v2, _v3));
    segments_.push_back(LineSegment(_v3, _v1));
  }

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const override;

  virtual CollisionObjectType getCollisionObjectType() const override {
    return CollisionObjectType::OBJ_TYPE_TRIANGLE;
  }

  Triangle *clone() const;

  void print(std::ostringstream &stream) const;

  ShapeType type() const;

  Eigen::Vector2d v1() const;
  Eigen::Vector2d v2() const;
  Eigen::Vector2d v3() const;
  Eigen::Vector2d center() const;
  double incircle_radius() const;
  Eigen::Vector2d incenter() const;

  double radius() const {
    assert(false);
    return 0;
  }

  void set_v1(const Eigen::Vector2d &_v1);
  void set_v2(const Eigen::Vector2d &_v2);
  void set_v3(const Eigen::Vector2d &_v3);

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif

  std::vector<LineSegment> segments(void) const { return segments_; };

 private:
  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &)
      const override;

  std::vector<LineSegment> segments_;

  Eigen::Vector2d compute_center();
  void compute_incircle_radius_and_center();

  using Shape::center_;
  using Shape::radius_;

  Eigen::Vector2d v1_;
  Eigen::Vector2d v2_;
  Eigen::Vector2d v3_;

  double incircle_radius_;
  Eigen::Vector2d incenter_;

  static constexpr ShapeType type_ = TYPE_TRIANGLE;
};

typedef std::shared_ptr<Triangle> TrianglePtr;
typedef std::shared_ptr<const Triangle> TriangleConstPtr;

}  // namespace collision

#endif
