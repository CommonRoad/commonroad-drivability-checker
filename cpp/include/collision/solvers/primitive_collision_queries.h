#ifndef PRIMITIVE_COLLISION_QUERIES_H_
#define PRIMITIVE_COLLISION_QUERIES_H_

#include <assert.h>
#include <math.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <iostream>

#include "collision/narrowphase/point.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/shape.h"
#include "collision/narrowphase/sphere.h"
#include "collision/narrowphase/triangle.h"

#include "collision/shape_group.h"
#include "collision/time_variant_collision_object.h"

namespace collision {
namespace solvers {
namespace solverPrimitive {

namespace primitive_queries {

#define COL_EPSILON (1e-05)

inline bool collisionDetection(const Point &point_first,
                               const Point &point_second) {
  Eigen::Vector2d d = point_first.center() - point_second.center();
  if (d.squaredNorm() <= COL_EPSILON) return true;
  return false;
}

inline bool collisionDetection(const Point &point, const RectangleAABB &aabb) {
  Eigen::Vector2d p = point.center() - aabb.center();
  if (fabs(p(0)) <= aabb.r_x() && fabs(p(1)) <= aabb.r_y()) return true;

  return 0;
}

inline bool collisionDetection(const Point &point, const Sphere &sphere) {
  Eigen::Vector2d p = point.center() - sphere.center();
  if (p.squaredNorm() <= pow(sphere.radius(), 2)) return true;
  return false;
}

// Computes if two AABBs collide.
// From: C. Ericson, Real-Time Collision Detection, pp. 80, 2004
//
// Return true if objects collide
inline bool collisionDetection(const RectangleAABB &aabb_first,
                               const RectangleAABB &aabb_second) {
  if (fabs(aabb_first.center_x() - aabb_second.center_x()) >
      (aabb_first.r_x() + aabb_second.r_x())) {
    return false;
  }
  if (fabs(aabb_first.center_y() - aabb_second.center_y()) >
      (aabb_first.r_y() + aabb_second.r_y())) {
    return false;
  }
  return true;
}

inline bool collisionDetection(const RectangleAABB &aabb,
                               const RectangleOBB &obb) {
  Eigen::Matrix2d aabb_axes;
  aabb_axes << 1, 0, 0, 1;

  // Compute rotation matrix expression second in first's coordinate frame
  Eigen::Matrix2d R = aabb_axes.transpose() * obb.local_axes();
  // Compute the translation vector between the boxes in first's coordinate
  // frame
  Eigen::Vector2d t = aabb_axes.transpose() * (obb.center() - aabb.center());

  // Project B onto A's axes
  for (int i = 0; i < 2; i++) {
    double r_first = aabb.r(i);
    double r_second = obb.r_x() * fabs(R(i, 0)) + obb.r_y() * fabs(R(i, 1));
    if (fabs(t(i)) > r_first + r_second) return false;
  }

  //! Project A onto B's axes
  for (int i = 0; i < 2; i++) {
    double r_first = aabb.r_x() * fabs(R(0, i)) + aabb.r_y() * fabs(R(1, i));
    double r_second = obb.r(i);
    double t_proj = fabs(t.transpose() * R.col(i));
    if (t_proj > r_first + r_second) return false;
  }

  return true;
}

// Computes if two OBBs collide.
// From: C. Ericson, Real-Time Collision Detection, pp. 103-105, 2004
//
// Return true if objects collide
inline bool collisionDetection(const RectangleOBB &obb_first,
                               const RectangleOBB &obb_second) {
  // Compute rotation matrix expression second in first's coordinate frame
  Eigen::Matrix2d R =
      obb_first.local_axes().transpose() * obb_second.local_axes();
  // Compute the translation vector between the boxes in first's coordinate
  // frame
  Eigen::Vector2d t = obb_first.local_axes().transpose() *
                      (obb_second.center() - obb_first.center());

  // Project B onto A's axes
  for (int i = 0; i < 2; i++) {
    double r_first = obb_first.r(i);
    double r_second =
        obb_second.r_x() * fabs(R(i, 0)) + obb_second.r_y() * fabs(R(i, 1));
    if (fabs(t(i)) > r_first + r_second) return false;
  }

  // Project A onto B's axes
  for (int i = 0; i < 2; i++) {
    double r_first =
        obb_first.r_x() * fabs(R(0, i)) + obb_first.r_y() * fabs(R(1, i));
    double r_second = obb_second.r(i);
    double t_proj = fabs(t.transpose() * R.col(i));
    if (t_proj > r_first + r_second) return false;
  }
  return true;
}

}  // namespace primitive_queries

}  // namespace solverPrimitive
}  // namespace solvers

}  // namespace collision

#endif
