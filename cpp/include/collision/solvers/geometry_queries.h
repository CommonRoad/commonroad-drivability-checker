#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_GEOMETRY_QUERIES_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_GEOMETRY_QUERIES_H_

#include "collision/collision_object.h"
#include "collision/narrowphase/detail/obb.h"

namespace collision

{
namespace geometry_queries {

CollisionObjectConstPtr ccd_merge_entities(const CollisionObject* first,
                                           const CollisionObject* second);

std::size_t test_polygon_enclosure(const ShapeGroup& sg_polygons,
                                   const RectangleOBB& obb, bool& res);

RectangleOBBConstPtr create_rectangle_obb_from_points(Eigen::Vector2d pt1,
                                                      Eigen::Vector2d pt2,
                                                      double rect_width);

int create_rectangles_obb_from_vertex_list(std::vector<Eigen::Vector2d>& verts,
                                           collision::ShapeGroup* sg_rects_ptr,
                                           double rect_width);

}  // namespace geometry_queries


namespace detail {
namespace geometry_queries {

OBB merge_obbs(const OBB& obb1, const OBB& obb2);

RectangleOBBConstPtr ccd_merge_entities(const RectangleOBB* first,
                                        const RectangleOBB* second);

int ccd_merge_entities(const Sphere* first,
                       const Sphere* second, ShapeGroupPtr ret);

}  // namespace geometry_queries
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_GEOMETRY_QUERIES_H_ */
