#pragma once
#include <Eigen/Dense>
#include "collision/narrowphase/triangle.h"

namespace collision {
namespace detail {
class Triangle_SAT2D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Triangle_SAT2D(const Triangle& tri) {
    corner.col(0) = tri.v1();
    corner.col(1) = tri.v2();
    corner.col(2) = tri.v3();

    // axes computation

    axes.col(0) = normal_vector(corner.col(1) - corner.col(0));
    axes.col(1) = normal_vector(corner.col(2) - corner.col(0));
    axes.col(2) = normal_vector(corner.col(2) - corner.col(1));

    for (int c1 = 0; c1 < 3; c1++) {
      axes.col(c1).normalize();
      origin[c1] = corner.col(origin_vert[c1]).dot(axes.col(c1));
      double dot_prod =
          corner.col(opposite_vert[c1]).dot(axes.col(c1)) - origin[c1];
      if (fabs(dot_prod) < 1e-10)  // projection is too small, invalid triangle
      {
        bValid = false;
        return;
      }

      axes.col(c1) /= dot_prod;
    }

    origin[0] = corner.col(0).dot(axes.col(0));
    origin[1] = corner.col(0).dot(axes.col(1));
    origin[2] = corner.col(1).dot(axes.col(2));

    bValid = true;
  }

  inline Eigen::Vector2d normal_vector(const Eigen::Vector2d& v) {
    return Eigen::Vector2d(v(1), -v(0));
  };

  Eigen::Matrix<double, 2, 3> corner;

  Eigen::Matrix<double, 2, 3> axes;

  double origin[3];

  bool bValid;

  static int opposite_vert[3];

  static int origin_vert[3];
};
}  // namespace detail
}  // namespace collision
