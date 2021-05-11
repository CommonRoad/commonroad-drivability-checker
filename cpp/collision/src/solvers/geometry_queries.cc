#include <fcl/math/bv/OBB.h>
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/shape_group.h"
#include "collision/solvers/boost/boost_collision_queries.h"

#include "collision/solvers/geometry_queries.h"

namespace collision {

namespace detail {
namespace geometry_queries {
int fillFclOBBHelper(const RectangleOBB* obb,
                     fcl::OBB<FCL_PRECISION>* fcl_obb) {
  fcl_obb->axis = fcl::Matrix3<FCL_PRECISION>::Zero();
  fcl_obb->To = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->extent = fcl::Vector3<FCL_PRECISION>::Zero();

  fcl_obb->To.topRows(2) = obb->center().cast<double>();

  if (obb->r_x() > obb->r_y())

  {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_x_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_y_axis();
    fcl_obb->axis(2, 2) = 1;

    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();

  } else {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_y_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_x_axis();
    fcl_obb->axis(2, 2) = 1;
    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();
  }

  return 0;
}

int fillFclOBBHelper(const OBB* obb, fcl::OBB<FCL_PRECISION>* fcl_obb) {
  fcl_obb->axis = fcl::Matrix3<FCL_PRECISION>::Zero();
  fcl_obb->To = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->extent = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->To.topRows(2) = obb->center().cast<double>();

  if (obb->r_x() > obb->r_y())

  {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_x_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_y_axis();
    fcl_obb->axis(2, 2) = 1;

    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();

  } else {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_y_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_x_axis();
    fcl_obb->axis(2, 2) = 1;
    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();
  }

  return 0;
}

// todo: add AABB handling

RectangleOBBConstPtr ccd_merge_entities(const RectangleOBB* first,
                                        const RectangleOBB* second) {
  fcl::OBB<FCL_PRECISION> this_obb;
  fcl::OBB<FCL_PRECISION> other_obb;

  fillFclOBBHelper(first, &this_obb);
  fillFclOBBHelper(second, &other_obb);

  fcl::OBB<FCL_PRECISION> merged_obb = this_obb + other_obb;

  Eigen::Matrix2d local_ax;

  local_ax = merged_obb.axis.block<2, 2>(0, 0).cast<double>();
  double rx = merged_obb.extent(0);
  double ry = merged_obb.extent(1);
  Eigen::Vector2d cent = merged_obb.To.topRows(2).cast<double>();

  RectangleOBBConstPtr ret(new collision::RectangleOBB(rx, ry, local_ax, cent));

  return (ret);
}

OBB merge_obbs(const OBB& first, const OBB& second) {
  OBB obb_merged;

  fcl::OBB<FCL_PRECISION> this_obb;
  fcl::OBB<FCL_PRECISION> other_obb;

  fillFclOBBHelper(&first, &this_obb);
  fillFclOBBHelper(&second, &other_obb);

  fcl::OBB<FCL_PRECISION> merged_obb = this_obb + other_obb;

  Eigen::Matrix2d local_ax;

  local_ax = merged_obb.axis.block<2, 2>(0, 0).cast<double>();
  Eigen::Vector2d r(merged_obb.extent(0), merged_obb.extent(1));

  Eigen::Vector2d cent = merged_obb.To.topRows(2).cast<double>();

  return OBB(local_ax, r, cent);
}

inline RectangleOBBConstPtr obb_from_aabb(const RectangleAABB* aabb) {
  return std::make_shared<const RectangleOBB>(aabb->r_x(), aabb->r_y(), 0,
                                              aabb->center());
}

}  // namespace geometry_queries
}  // namespace detail
namespace geometry_queries {
CollisionObjectConstPtr ccd_merge_entities(const CollisionObject* first,
                                           const CollisionObject* second) {
  if ((first->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) ||
      (second->getCollisionObjectType() == OBJ_TYPE_AABB_BOX)) {
    RectangleOBBConstPtr old_rect1;
    RectangleOBBConstPtr old_rect2;
    if (first->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) {
      old_rect1 = detail::geometry_queries::obb_from_aabb(
          static_cast<const RectangleAABB*>(first));
      first = old_rect1.get();
    }
    if (second->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) {
      old_rect2 = detail::geometry_queries::obb_from_aabb(
          static_cast<const RectangleAABB*>(second));
      second = old_rect2.get();
    }
    return detail::geometry_queries::ccd_merge_entities(
        static_cast<const RectangleOBB*>(first),
        static_cast<const RectangleOBB*>(second));
  }

  if (first->getCollisionObjectType() == OBJ_TYPE_OBB_BOX &&
      second->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
    return detail::geometry_queries::ccd_merge_entities(
        static_cast<const RectangleOBB*>(first),
        static_cast<const RectangleOBB*>(second));
  }
  /*
            else if(first->getCollisionObjectType()==OBJ_TYPE_POLYGON &&
     second->getCollisionObjectType()==OBJ_TYPE_POLYGON)
            {
                    PolygonConstPtr new_poly;
                    int res=boost_poly_poly_convex_hull(*(static_cast<const
     Polygon*>(first.get())),*(static_cast<const
     Polygon*>(second.get())),new_poly); if(res) return nullptr; return
     new_poly;
            }
   */
  else
    return nullptr;
}

std::size_t test_polygon_enclosure(const ShapeGroup& sg,
                                   const RectangleOBB& obb, bool& res) {
  if (boost_polygon_enclosure(sg, obb, res)) {
    return 1;
  } else
    return 0;
}

RectangleOBBConstPtr create_rectangle_obb_from_points(Eigen::Vector2d pt1,
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

int create_rectangles_obb_from_vertex_list(std::vector<Eigen::Vector2d>& verts,
                                           collision::ShapeGroup* sg_rects_ptr,
                                           double rect_width) {
  for (auto vert = verts.begin(); vert < verts.end() - 1; vert++) {
    collision::RectangleOBBConstPtr rect_new =
        create_rectangle_obb_from_points(*vert, *(vert + 1), rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  if (verts.size() > 1) {
    collision::RectangleOBBConstPtr rect_new = create_rectangle_obb_from_points(
        *(verts.end() - 1), *(verts.begin()), rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  return 0;
}

}  // namespace geometry_queries
}  // namespace collision
