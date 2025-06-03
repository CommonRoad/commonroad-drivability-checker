#ifndef COLLISION_FCL_FCL_TRANSFORM_H_
#define COLLISION_FCL_FCL_TRANSFORM_H_

#include <Eigen/Dense>

#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"

namespace collision {
namespace solvers {
namespace solverFCL {
class FCLTransform {
 public:
  static double fcl_get_radian_angle(const Eigen::Vector2d &Axis) {
    double norm = Axis.norm();
    double x_norm = Axis.x() / norm;
    double angle = std::acos(x_norm);
    if (Axis.y() < 0) angle *= -1;
    return angle;
  }
  static fcl::Transform3<FCL_PRECISION> fcl_get_3d_translation(
      const Eigen::Vector2d &pos) {
    return fcl::Transform3<FCL_PRECISION>(fcl::Translation3<FCL_PRECISION>(
        fcl::Vector3<FCL_PRECISION>(pos.x(), pos.y(), 0)));
  }

  static fcl::Transform3<FCL_PRECISION> fcl_get_3d_rotation_translation(
      const Eigen::Vector2d &pos, const Eigen::Vector2d &x_axis) {
    fcl::Transform3<FCL_PRECISION> trans = fcl_get_3d_translation(pos);
    trans.rotate(fcl::AngleAxis<FCL_PRECISION>(
        fcl_get_radian_angle(x_axis), fcl::Vector3<FCL_PRECISION>::UnitZ()));
    return trans;
  }
  static fcl::Transform3<FCL_PRECISION> fcl_get_3d_rotation_translation(
      const Eigen::Vector2d &pos, double angle) {
    fcl::Transform3<FCL_PRECISION> trans = fcl_get_3d_translation(pos);
    trans.rotate(fcl::AngleAxis<FCL_PRECISION>(
        angle, fcl::Vector3<FCL_PRECISION>::UnitZ()));
    return trans;
  }
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
#endif /* COLLISION_FCL_FCL_TRANSFORM_H_ */
