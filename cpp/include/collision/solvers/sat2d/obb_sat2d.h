#pragma once
#include <Eigen/Dense>

#include "collision/solvers/sat2d/triangle_sat2d.h"

// Inspired by https://flipcode.com/archives/2D_OBB_Intersection.shtml

namespace collision {
namespace detail {
class Triangle_SAT2D;

class OBB_SAT2D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OBB_SAT2D(const Eigen::Vector2d& center, const double w, const double h,
            const Eigen::Vector2d& X_axis, const Eigen::Vector2d& Y_axis) {
    Eigen::Vector2d X = X_axis;
    Eigen::Vector2d Y = Y_axis;

    X *= w / 2;
    Y *= h / 2;

    corner.col(0) = center - X - Y;
    corner.col(1) = center + X - Y;
    corner.col(2) = center + X + Y;
    corner.col(3) = center - X + Y;

    axes.col(0) = corner.col(1) - corner.col(0);
    axes.col(1) = corner.col(3) - corner.col(0);

    for (int c1 = 0; c1 < 2; c1++) {
      axes.col(c1) /= axes.col(c1).squaredNorm();
      origin[c1] = corner.col(0).dot(axes.col(c1));
    }
  }

  bool overlaps(const OBB_SAT2D& other) const {
    return overlaps1Way(other) && other.overlaps1Way(*this);
  }

  bool overlaps1Way(const OBB_SAT2D& other) const {
    for (int c1 = 0; c1 < 2; c1++) {
      double t = other.corner.col(0).dot(axes.col(c1));

      double min_t = t;
      double max_t = t;

      for (int c2 = 1; c2 < 4; ++c2) {
        t = other.corner.col(c2).dot(axes.col(c1));

        if (t < min_t) {
          min_t = t;
        } else if (t > max_t) {
          max_t = t;
        }
      }

      if ((min_t > 1 + origin[c1]) || (max_t < origin[c1])) {
        return false;
      }
    }

    return true;
  }

  Eigen::Matrix<double, 2, 4> corner;

  Eigen::Matrix2d axes;

  double origin[2];
};
}  // namespace detail
}  // namespace collision
