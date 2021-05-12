#ifndef CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_DETAIL_OBB_H_
#define CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_DETAIL_OBB_H_

#include <Eigen/Dense>

#include "collision/narrowphase/detail/aabb.h"

namespace collision {
namespace detail {
class OBB {
 public:
  OBB(const Eigen::Matrix2d& local_axes, const Eigen::Vector2d& r,
      const Eigen::Vector2d& center)
      : local_axes_(local_axes), r_(r), center_(center) {
    return;
  }

  OBB(void) { return; }

  int set_local_axes(const Eigen::Matrix2d& local_axes) {
    local_axes_ = local_axes;
    return 0;
  }

  int set_r(const Eigen::Vector2d& r) {
    r_ = r;
    return 0;
  }

  int set_center(const Eigen::Vector2d& center) {
    center_ = center;
    return 0;
  }

  Eigen::Matrix2d local_axes(void) const { return local_axes_; }

  Eigen::Vector2d r(void) const { return r_; }

  Eigen::Vector2d center(void) const { return center_; }

  Eigen::Vector2d local_x_axis() const { return local_axes_.col(0); }

  Eigen::Vector2d local_y_axis() const { return local_axes_.col(1); }

  AABB getAABB() const {
    Eigen::Vector2d _v1 =
        center() - r_x() * local_x_axis() + r_y() * local_y_axis();
    Eigen::Vector2d _v2 =
        center() + r_x() * local_x_axis() + r_y() * local_y_axis();
    Eigen::Vector2d _v3 =
        center() + r_x() * local_x_axis() - r_y() * local_y_axis();
    Eigen::Vector2d _v4 =
        center() - r_x() * local_x_axis() - r_y() * local_y_axis();

    AABB aabb;

    double min_x = std::min(_v1(0), _v2(0));
    double tmp = std::min(_v3(0), _v4(0));
    aabb.x_min = std::min(min_x, tmp);

    double min_y = std::min(_v1(1), _v2(1));
    tmp = std::min(_v3(1), _v4(1));
    aabb.y_min = std::min(min_y, tmp);

    double max_x = std::max(_v1(0), _v2(0));
    tmp = std::max(_v3(0), _v4(0));
    aabb.x_max = std::max(max_x, tmp);

    double max_y = std::max(_v1(1), _v2(1));
    tmp = std::max(_v3(1), _v4(1));
    aabb.y_max = std::max(max_y, tmp);

    return aabb;
  }

  double r_x() const { return r_(0); }

  double r_y() const { return r_(1); }

 private:
  Eigen::Matrix2d local_axes_;
  Eigen::Vector2d r_;
  Eigen::Vector2d center_;
};
}  // namespace detail
};  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_DETAIL_OBB_H_ */
