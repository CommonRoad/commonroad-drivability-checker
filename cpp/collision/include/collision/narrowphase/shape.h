#ifndef SHAPE_H_
#define SHAPE_H_

#include <Eigen/Dense>
#include <iostream>

#include "collision/collision_object_ex.h"
#include "collision/solvers/boost/i_boost_collision_object.h"
#include "collision/solvers/fcl/i_fcl_collision_object.h"

namespace collision {

enum ShapeType {
  TYPE_POINT,
  TYPE_RECTANGLE_AABB,
  TYPE_RECTANGLE_OBB,
  TYPE_SPHERE,
  TYPE_TRIANGLE,
  TYPE_POLYGON
};

typedef ShapeType ShapeType;

//! Base prototype for the shape of an obstacle
class Shape : public CollisionObjectEx, IFCLCollisionObject {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionObjectClass getCollisionObjectClass() const override {
    return CollisionObjectClass::OBJ_CLASS_SHAPE;
  }

  virtual const ISolverEntity_FCL *getFclInterface() const override {
    return this;
  }

  Shape(const Shape &copy);
  virtual Shape *clone() const = 0;

  //! Get geometric center of shape
  Eigen::Vector2d center() const;
  double center_x() const;
  double center_y() const;

  //! Set geometric center of shape
  void set_center(const Eigen::Vector2d &_center);

  //! Print all parameters of the shape
  virtual void print(std::ostringstream &stream) const = 0;
  virtual CollisionObjectConstPtr timeSlice(
      int time_idx, CollisionObjectConstPtr shared_ptr_this) const;

  //! Get shape type
  virtual ShapeType type() const = 0;

  //! Get radius
  double radius() const;

  // virtual ShapeConstPtr scale(double factor) const = 0;
  // virtual ShapeConstPtr shrink(double value) const = 0;

  virtual ~Shape();

#if ENABLE_SERIALIZER
  virtual serialize::ICollisionObjectExport *exportThis(void) const = 0;
#endif

 protected:
  Shape(const Eigen::Vector2d &_center) : center_(_center) {}

 protected:
  Eigen::Vector2d center_;
  double radius_;
};

typedef std::shared_ptr<const Shape> ShapeConstPtr;

}  // namespace collision

#endif
