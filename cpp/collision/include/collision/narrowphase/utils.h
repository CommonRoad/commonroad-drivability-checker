#ifndef CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_UTILS_H_
#define CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_UTILS_H_

#include "collision/narrowphase/rectangle_obb.h"
#include "collision/shape_group.h"

namespace collision {

RectangleOBBConstPtr create_obb_from_points(Eigen::Vector2d pt1,
                                            Eigen::Vector2d pt2,
                                            double rect_width) {
  Eigen::Matrix2d local_axes;

  Eigen::Vector2d v = pt2 - pt1;

  Eigen::Vector2d center = (pt1 + pt2) / 2;

  double length = v.norm();

  if (length < 1e-10) return 0;

  v = v / length;

  local_axes.col(0) = v;
  local_axes.col(1) = Eigen::Vector2d(-v(1), v(0));
  return std::make_shared<const RectangleOBB>(length / 2, rect_width / 2,
                                              local_axes, center);
}

int generate_rectangles_from_vertex_list(std::vector<Eigen::Vector2d> &verts,
                                         collision::ShapeGroup *sg_rects_ptr,
                                         double rect_width) {
  for (auto vert = verts.begin(); vert < verts.end() - 1; vert++) {
    collision::RectangleOBBConstPtr rect_new =
        collision::create_obb_from_points(*vert, *(vert + 1), rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  if (verts.size() > 1) {
    collision::RectangleOBBConstPtr rect_new =
        collision::create_obb_from_points(*(verts.end() - 1), *(verts.begin()),
                                          rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  return 0;
}
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_NARROWPHASE_UTILS_H_ */
