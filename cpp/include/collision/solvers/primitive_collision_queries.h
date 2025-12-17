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

// Compute the 2D pseudo cross product Dot(Perp(u), v)
inline float cross2D(const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
  return u(1)*v(0) - u(0)*v(1);
}

inline bool isTriangleValid(const Triangle& tri) {
  return tri.is_valid();
}

// experimental
// Test if 2D point P lies inside the counterclockwise 2D triangle ABC
inline bool collisionDetection(const Point& point,
                               const Triangle& triangle) {

  // check if it is a valid triangle
  if (!isTriangleValid(triangle))
	  return false;

  // Translate point and triangle so that point lies at origin
  Eigen::Vector2d pa = point.center()-triangle.v1();
  Eigen::Vector2d pb = point.center()-triangle.v2();
  Eigen::Vector2d ba = triangle.v2()-triangle.v1();
  Eigen::Vector2d cb = triangle.v3()-triangle.v2();

  float pab = cross2D(pa,ba);
  float pbc = cross2D(pb,cb);
  if(std::signbit(pab) != std::signbit(pbc))
    return false;

  Eigen::Vector2d pc = point.center()-triangle.v3();
  Eigen::Vector2d ac = triangle.v1()-triangle.v3();
  float pca = cross2D(pc,ac);
  if(std::signbit(pab) != std::signbit(pca))
    return false;

  return true;
}

// experimental
inline bool collisionDetection(const Point& point,
                               const RectangleOBB& obb) {
  Eigen::Vector2d pt1(obb.center() - obb.r_x() * obb.local_x_axis() -
                      obb.r_y() * obb.local_y_axis());
  Eigen::Vector2d pt2(obb.center() + obb.r_x() * obb.local_x_axis() -
                      obb.r_y() * obb.local_y_axis());
  Eigen::Vector2d pt3(obb.center() + obb.r_x() * obb.local_x_axis() +
                      obb.r_y() * obb.local_y_axis());
  Eigen::Vector2d pt4(obb.center() - obb.r_x() * obb.local_x_axis() +
                      obb.r_y() * obb.local_y_axis());
  Triangle triangle_1(pt1, pt2, pt3);
  Triangle triangle_2(pt3, pt4, pt1);
  return collisionDetection(point, triangle_1) ||
          collisionDetection(point, triangle_2);
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

// experimental
// Computes if a sphere and an AABB collide.
// From: C. Ericson, Real-Time Collision Detection, pp. 165-166, 2004
//
// Return true if objects collide
inline bool collisionDetection(const RectangleAABB& aabb,
                               const Sphere& sphere) {
  double sq_dis = aabb.squareDisToPoint(sphere.center());
  // Collision if squared distance between the center of the sphere and AABB
  //  is less than the radius of the sphere.
  if(sq_dis <= pow(sphere.radius(),2))
      return true;

  return false;
}

// experimental
inline bool collisionDetection(const RectangleAABB& aabb,
                               const Triangle& triangle) {

  if (!isTriangleValid(triangle)) {
	  return false;
  }

  Eigen::Vector2d v1 = triangle.v1()-aabb.center();
  Eigen::Vector2d v2 = triangle.v2()-aabb.center();
  Eigen::Vector2d v3 = triangle.v3()-aabb.center();

  // two separating axis of the rectangle
  if (v1(0) < -aabb.r_x() &&
      v2(0) < -aabb.r_x() &&
      v3(0) < -aabb.r_x()) {
    return false;
  }
  if (v1(0) > aabb.r_x() &&
      v2(0) > aabb.r_x() &&
      v3(0) > aabb.r_x()) {
    return false;
  }
  if (v1(1) < -aabb.r_y() &&
      v2(1) < -aabb.r_y() &&
      v3(1) < -aabb.r_y()) {
    return false;
  }
  if (v1(1) > aabb.r_y() &&
      v2(1) > aabb.r_y() &&
      v3(1) > aabb.r_y()) {
    return false;
  }

  // three separating axis of the triangle
  Eigen::Vector2d a; // axis equation coefficient
  double d; // axis equation distance

  Eigen::Vector2d r1(-aabb.r_x(), -aabb.r_y());
  Eigen::Vector2d r2(+aabb.r_x(), -aabb.r_y());
  Eigen::Vector2d r3(+aabb.r_x(), +aabb.r_y());
  Eigen::Vector2d r4(-aabb.r_x(), +aabb.r_y());

  // axis perpendicular to v1->v2
  a(0) = v2(1)-v1(1);
  a(1) = -(v2(0)-v1(0));
  d = a.transpose()*v1;
  if (a.transpose()*v3 > d) {
    d = -d;
  }

  if (a.transpose()*r1 > d &&
      a.transpose()*r2 > d &&
      a.transpose()*r3 > d &&
      a.transpose()*r4 > d) {
      return false;
  }

  // axis perpendicular to v2->v3
  a(0) = v3(1)-v2(1);
  a(1) = -(v3(0)-v2(0));
  d = a.transpose()*v2;
  if (a.transpose()*v1 > d) {
    d = -d;
  }

  if (a.transpose()*r1 > d &&
      a.transpose()*r2 > d &&
      a.transpose()*r3 > d &&
      a.transpose()*r4 > d) {
    return false;
  }

  // axis perpendicular to v3->v1
  a(0) = v1(1)-v3(1);
  a(1) = -(v1(0)-v3(0));
  d = a.transpose()*v3;
  if (a.transpose()*v2 > d) {
    d = -d;
  }

  if (a.transpose()*r1 > d &&
      a.transpose()*r2 > d &&
      a.transpose()*r3 > d &&
      a.transpose()*r4 > d) {
    return false;
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

// experimental
// Computes if a sphere and an OBB collide.
// From: C. Ericson, Real-Time Collision Detection, pp. 166-167, 2004
//
// Return true if objects collide
inline bool collisionDetection(const RectangleOBB& obb, const Sphere& sphere) {
  double sq_dis = obb.squareDisToPoint(sphere.center());
  // Collision if squared distance between the center of the sphere and AABB
  //  is less than the radius of the sphere.
  if(sq_dis <= pow(sphere.radius(),2))
    return true;

return false;
}

// experimental
// Computes if two spheres collide.
// From: C. Ericson, Real-Time Collision Detection, pp. 88, 2004
//
// Return true if objects collide
inline bool collisionDetection(const Sphere& sphere_first,
                               const Sphere& sphere_second) {
  double dis = pow((sphere_first.center_x() - sphere_second.center_x()),2) +
               pow((sphere_first.center_y() - sphere_second.center_y()),2);
  // Spheres intersect if squared distance between centers is less than squared
  // sum of radii
  double sum_radii = sphere_first.radius() + sphere_second.radius();
  return dis <= pow(sum_radii,2);
}

inline Eigen::Vector2d closestPointOnLineSegment(const Eigen::Vector2d &l_a,
                                                 const Eigen::Vector2d &l_b,
                                                 const Eigen::Vector2d &pt) {
  Eigen::Vector2d a = pt - l_a;
  Eigen::Vector2d b = l_b - l_a;
  double r = a.dot(b) / b.dot(b);
  r = std::max(0.0, r);
  r = std::min(1.0, r);
  return l_a + r * b;
}

inline bool pointInsideTriangle(const Triangle& triangle,
                                const Eigen::Vector2d &pt) {

  auto circle_point_in_same_halfspace = [&] (const Eigen::Vector2d &l_a,
                                             const Eigen::Vector2d &l_b,
                                             const Eigen::Vector2d &pt_1,
                                             const Eigen::Vector2d &pt_2)
                                             -> bool {
    auto t = l_b - l_a;
    auto n = Eigen::Vector2d(t(1), -t(0));
    double d_pt_1, d_pt_2, d;
    d_pt_1 = n.dot(pt_1);
    d_pt_2 = n.dot(pt_2);
    d = n.dot(l_a);
    if (((d_pt_1 <= d) && (d_pt_2 <= d)) ||
        ((d <= d_pt_1) && (d <= d_pt_2))) {
      return true;
    } else {
      return false;
    }
  };

  if (circle_point_in_same_halfspace(triangle.v1(), triangle.v2(),
                                     triangle.v3(), pt) &&
      circle_point_in_same_halfspace(triangle.v2(), triangle.v3(),
                                     triangle.v1(), pt) &&
      circle_point_in_same_halfspace(triangle.v3(), triangle.v1(),
                                     triangle.v2(), pt)) {
    return true;
  } else {
    return false;
  }
}

// experimental
inline bool collisionDetection(const Sphere& sphere, const Triangle& triangle) {
  if (!isTriangleValid(triangle))
    return false;

  if (pointInsideTriangle(triangle, sphere.center())) {
    return true;
  }

  Eigen::Vector2d vector_center_to_closet_point = (
    closestPointOnLineSegment(triangle.v1(), triangle.v2(),
                              sphere.center()) - sphere.center());
  if (vector_center_to_closet_point.dot(vector_center_to_closet_point) <=
      sphere.radius() * sphere.radius()) {
    return true;
  }

  vector_center_to_closet_point = (
    closestPointOnLineSegment(triangle.v2(), triangle.v3(),
                              sphere.center()) - sphere.center());
  if (vector_center_to_closet_point.dot(vector_center_to_closet_point) <=
    sphere.radius() * sphere.radius()) {
    return true;
  }

  vector_center_to_closet_point = (
      closestPointOnLineSegment(triangle.v3(), triangle.v1(),
                                sphere.center()) -
                                sphere.center());
  if (vector_center_to_closet_point.dot(vector_center_to_closet_point) <=
      sphere.radius() * sphere.radius()) {
      return true;
  }

  return false;
}

// experimental
inline bool collisionDetection(const Triangle& triangle_a,
                               const Triangle& triangle_b) {

  if (!isTriangleValid(triangle_a) || !isTriangleValid(triangle_b))
    return false;

  Eigen::Vector2d n;
  std::array<double, 6> projected_values = {0,0,0,0,0,0};

  auto normal_vector = [] (const Eigen::Vector2d& v) ->  Eigen::Vector2d {
    return Eigen::Vector2d(v(1), -v(0));
  };

  auto project_vertices = [&] () {
    projected_values[0] = n.transpose()*triangle_a.v1();
    projected_values[1] = n.transpose()*triangle_a.v2();
    projected_values[2] = n.transpose()*triangle_a.v3();
    projected_values[3] = n.transpose()*triangle_b.v1();
    projected_values[4] = n.transpose()*triangle_b.v2();
    projected_values[5] = n.transpose()*triangle_b.v3();
  };

  auto separated = [&] () -> bool {
    double min_t1, max_t1, min_t2, max_t2;
    min_t1 = std::min(std::min(projected_values[0], projected_values[1]),
                      projected_values[2]);
    max_t1 = std::max(std::max(projected_values[0], projected_values[1]),
                      projected_values[2]);
    min_t2 = std::min(std::min(projected_values[3], projected_values[4]),
                      projected_values[5]);
    max_t2 = std::max(std::max(projected_values[3], projected_values[4]),
                      projected_values[5]);
    return min_t1>max_t2 || min_t2>max_t1;
  };

  // separating axes of triangle 1
  // axis perpendicular to v1->v2
  n = normal_vector(triangle_a.v2()-triangle_a.v1());
  project_vertices();
  if (separated()) { return false; }

  // axis perpendicular to v2->v3
  n = normal_vector(triangle_a.v3()-triangle_a.v2());
  project_vertices();
  if (separated()) { return false; }

  // axis perpendicular to v3->v1
  n = normal_vector(triangle_a.v1()-triangle_a.v3());
  project_vertices();
  if (separated()) { return false; }

  // separating axes of triangle 2
  // axis perpendicular to v1->v2
  n = normal_vector(triangle_b.v2()-triangle_b.v1());
  project_vertices();
  if (separated()) { return false; }

  // axis perpendicular to v2->v3
  n = normal_vector(triangle_b.v3()-triangle_b.v2());
  project_vertices();
  if (separated()) { return false; }

  // axis perpendicular to v3->v1
  n = normal_vector(triangle_b.v1()-triangle_b.v3());
  project_vertices();
  if (separated()) { return false; }

  return true;
}

// experimental
inline bool collisionDetection(const RectangleOBB& obb,
                               const Triangle& triangle) {
  Eigen::Vector2d pt1(obb.center() - obb.r_x() * obb.local_x_axis() -
                      obb.r_y() * obb.local_y_axis());
  Eigen::Vector2d pt2(obb.center() + obb.r_x() * obb.local_x_axis() -
                      obb.r_y()*obb.local_y_axis());
  Eigen::Vector2d pt3(obb.center() + obb.r_x() * obb.local_x_axis() +
                      obb.r_y() * obb.local_y_axis());
  Eigen::Vector2d pt4(obb.center() - obb.r_x() * obb.local_x_axis() +
                      obb.r_y() * obb.local_y_axis());
  Triangle triangle_1(pt1, pt2, pt3);
  Triangle triangle_2(pt3, pt4, pt1);
  return collisionDetection(triangle, triangle_1) ||
         collisionDetection(triangle, triangle_2);
}

}  // namespace primitive_queries

}  // namespace solverPrimitive
}  // namespace solvers

}  // namespace collision

#endif
