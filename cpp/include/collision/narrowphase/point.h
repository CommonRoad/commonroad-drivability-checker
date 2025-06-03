#ifndef POINT_H_
#define POINT_H_

#include <Eigen/Dense>
#include <iostream>

#include <vector>

#include "collision/narrowphase/shape.h"

namespace collision {

#define COLLISION_FCL_POINT_EPS 0.000001

/*!
        \brief A point in space
*/
class Point : public Shape {
 public:
  Point(const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0))
      : Shape(_center) {}

  ~Point() {}

  Point(const Point &copy);
  Point *clone() const;

  void print(std::ostringstream &stream) const;

  virtual CollisionObjectType getCollisionObjectType() const override {
    return CollisionObjectType::OBJ_TYPE_POINT;
  }

  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &)
      const override;

  ShapeType type() const;

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif

 private:
  using Shape::center_;
  using Shape::radius_;

  static constexpr ShapeType type_ = TYPE_POINT;
};

}  // namespace collision

#endif
