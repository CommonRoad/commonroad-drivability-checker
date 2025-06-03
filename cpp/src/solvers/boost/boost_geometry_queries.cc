
#include "collision/shape_group.h"
#include "collision/solvers/boost/boost_helpers.h"
#include "collision/solvers/boost/boost_object_polygon.h"

#include "collision/solvers/boost/boost_geometry_queries.h"

namespace collision {
namespace solvers {
namespace solverBoost {
collision::ShapeGroupPtr lane_polygons_postprocess(
    collision::ShapeGroup &sg_polys, double buf_width, bool triangulate) {
  collision::ShapeGroupPtr sg_ret = std::make_shared<collision::ShapeGroup>();
  auto sg_ret_ptr = sg_ret.get();

  std::vector<collision::BoostPolygon *> candidate_polygons_boost;

  for (auto obj : sg_polys.unpack()) {
    if (obj->getCollisionObjectType() == collision::OBJ_TYPE_POLYGON) {
      auto poly_ptr = static_cast<const collision::Polygon *>(obj.get());
      candidate_polygons_boost.push_back(static_cast<collision::BoostPolygon *>(
          collision::solvers::solverBoost::get_boost_object_ptr(poly_ptr)
              ->getCollisionObject_boost()
              .get()));
    } else {
      throw std::invalid_argument(
          "The shape group for the postprocessing of lane polygons can contain "
          "only collision::Polygon objects");
    }
  }

  collision::triangulation::TriangulationQuality qual;

  qual.bb_only = !triangulate;

  for (auto boost_poly_ptr : candidate_polygons_boost) {
    std::vector<collision::BoostPolygon> output;
    if (buf_width) {
      boost_poly_ptr->extend(output, buf_width);
      for (auto poly : output) {
        sg_ret_ptr->addToGroup(poly.toPolygon(qual));
      }
    } else {
      sg_ret_ptr->addToGroup(boost_poly_ptr->toPolygon(qual));
    }
  }
  return sg_ret;
}
}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision
