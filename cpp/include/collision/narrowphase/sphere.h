#ifndef SPHERE_H_
#define SPHERE_H_

#include <Eigen/Dense>
#include <iostream>

#include "collision/narrowphase/shape.h"

namespace collision {
/*!
  \brief Circle

*/
class Sphere : public Shape {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sphere(double radius, const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0))
      : Shape(_center) {
    radius_ = radius;
  }

  Sphere(double radius, double x, double y) : Shape(Eigen::Vector2d(x, y)) {
    radius_ = radius;
  }

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const override;

  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &)
      const override;

  ~Sphere() {}

  virtual CollisionObjectType getCollisionObjectType() const override {
    return CollisionObjectType::OBJ_TYPE_SPHERE;
  };

  Sphere(const Sphere &copy);
  Sphere *clone() const;

  void print(std::ostringstream &stream) const;

  void set_radius(double _radius);
  double radius() const { return radius_; };
  double get_x() const { return center_(0); };
  double get_y() const { return center_(1); };

  ShapeType type() const;

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif

 private:
  using Shape::center_;
  using Shape::radius_;

  static constexpr ShapeType type_ = TYPE_SPHERE;
};

}  // namespace collision

#endif
