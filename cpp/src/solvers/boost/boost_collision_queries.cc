#include "collision/solvers/boost/boost_collision_queries.h"
#include "collision/solvers/geometry_queries.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/shape_group.h"
#include "collision/solvers/accelerators/detail/container_grid.h"

namespace collision {

using namespace detail::accelerators;

namespace solvers {
namespace solverBoost {

#define BOOST_EPS (0)

typedef boost::geometry::model::multi_polygon<bg_polygon_type> bg_mpolygon_type;

/*!
        \brief Returns true if the BoostPolygon polyg1 is completely contained
   within the other BoostPolygon object polyg2.
*/

bool boost_within(const BoostPolygon& polyg1, const BoostPolygon& polyg2) {
  return polyg1.within(polyg2);
}

/*!
        \brief Creates a collision Polygon of two RectangleOBB objects. The
   convex hull of the boolean union of the oriented rectangles is computed.

        Returns 0 on successful creation.

         \param[in] pos1 OBB rectangle 1

         \param[in] pos2 OBB rectangle 2

         \param[out] resPoly output collision Polygon

*/

int boost_poly_poly_convex_hull(const Polygon& pos1, const Polygon& pos2,
                                collision::PolygonConstPtr& resPoly) {
  BoostPolygon bp1(&pos1);
  BoostPolygon bp2(&pos2);
  std::vector<bg_polygon_type> output;
  bg_polygon_type* boostp_1 = bp1.getInternal();
  bg_polygon_type* boostp_2 = bp2.getInternal();

  {
    // STACK_TIMER timer_union(TIMER_union);
    boost::geometry::union_(*boostp_1, *boostp_2, output);
  }
  if (output.size() != 1) {
    return -1;
  }

  // convex Hull
  {
    // STACK_TIMER timer_chull(TIMER_chull_polygon);
    resPoly = BoostPolygon(output[0]).convexHull()->toPolygon();
  }
  return 0;
}

inline int boost_obb_obb_convex_hull(const RectangleOBB& pos1,
                                     const RectangleOBB& pos2,
                                     collision::PolygonConstPtr& resPoly) {
  BoostPolygon bp1(&pos1);
  BoostPolygon bp2(&pos2);
  std::vector<bg_polygon_type> output;
  bg_polygon_type* boostp_1 = bp1.getInternal();
  bg_polygon_type* boostp_2 = bp2.getInternal();

  {
    // STACK_TIMER timer_union(TIMER_union);
    boost::geometry::union_(*boostp_1, *boostp_2, output);
  }
  if (output.size() != 1) {
    return -1;
  }

  // convex Hull
  {
    // STACK_TIMER timer_chull(TIMER_chull_polygon);
    resPoly = BoostPolygon(output[0]).convexHull()->toPolygon();
  }
  return 0;
}

/*!
        \brief Checks if the convex hull of the Boolean union of two given OBB
   boxes collides with any of the ShapeGroup objects.


         Returns 0 if no error has occured.

         \param[in] sg ShapeGroup with static obstacles

         \param[in] pos1 OBB rectangle 1

         \param[in] pos2 OBB rectangle 2

         \param[out] res Boolean collision result

*/

int boost_ccd_convex_hull_collision(const ShapeGroup& sg,
                                    const RectangleOBB& pos1,
                                    const RectangleOBB& pos2, bool& res) {
  collision::PolygonConstPtr resPoly;
  int err = boost_obb_obb_convex_hull(pos1, pos2, resPoly);
  if (err) return err;
  {
    // STACK_TIMER timer_collide(TIMER_collide_3);
    res = sg.collide(*resPoly);
  }
  return 0;
}

int boost_ccd_obb_sum_collision(const ShapeGroup& sg, const RectangleOBB& pos1,
                                const RectangleOBB& pos2, bool& res) {
  collision::RectangleOBBConstPtr resOBB =
      detail::geometry_queries::ccd_merge_entities(&pos1, &pos2);
  {
    // STACK_TIMER timer_collide(TIMER_collide_3);
    res = sg.collide(*resOBB);
  }
  return 0;
}

inline int boost_get_candidate_polygons_no_merge(
    const ShapeGroup& sg, const Shape& shape,
    std::vector<bg_polygon_type>& mpoly) {
  bg_mpolygon_type mpoly_res;
  bg_mpolygon_type tmp_union;

  for (auto obj : sg.unpack()) {
    if (obj->getCollisionObjectType() !=
        collision::CollisionObjectType::OBJ_TYPE_POLYGON) {
      return -1;
    }
    const collision::Polygon* obj_poly =
        static_cast<const collision::Polygon*>(obj.get());
    if (obj_poly->getAABB()->collide(shape)) {
      BoostPolygon bp(obj_poly);
      bg_polygon_type poly_tmp = *(bp.getInternal());

      mpoly.push_back(poly_tmp);
    }
  }
  return 0;
}

int boost_polygon_enclosure(const ShapeGroup& sg, const RectangleOBB& obb,
                            bool& res) {
  BoostPolygon bp1(&obb);
  std::vector<bg_polygon_type> poly_vec;
  int err = 0;
  {
    // STACK_TIMER timer_get_cand(TIMER_get_cand);
    err = boost_get_candidate_polygons_no_merge(sg, obb, poly_vec);
    if (err) return err;
  }

  bg_mpolygon_type output;
  bg_polygon_type cur_poly = *(bp1.getInternal());
  output.push_back(cur_poly);
  bg_mpolygon_type tmp_union;
  {
    // STACK_TIMER timer_difference(TIMER_difference);
    for (const auto &polygon : poly_vec) {
      boost::geometry::clear(tmp_union);
      boost::geometry::difference(output, polygon, tmp_union);
      output = tmp_union;
    }
  }
  if (boost::geometry::is_empty(output) ||
      ((BOOST_EPS > 0) && boost::geometry::area(output) < BOOST_EPS)) {
    res = true;
  } else {
    res = false;
  }
  return 0;
}

template <typename T>
int boost_polygon_enclosure_grid(T& grid, std::vector<BoostPolygon*> polygons,
                                 AABB& aabb, const BoostPolygon* obj_poly,
                                 bool& res) {
  STACK_TIMER tim(TIMER_polygon_enclosure);
  aligned_vector<int> candidates;
  std::vector<bg_polygon_type> poly_vec;

  detail::accelerators::windowQuery(grid, aabb, candidates);

  for (auto el : candidates) {
    poly_vec.push_back(*(polygons[el]->getInternal()));
  }

  bg_mpolygon_type output;
  bg_polygon_type cur_poly = *(obj_poly->getInternal());
  output.push_back(cur_poly);
  bg_mpolygon_type tmp_union;
  {
    STACK_TIMER timer_difference(TIMER_difference);
    for (const auto &polygon : poly_vec) {
      boost::geometry::clear(tmp_union);
      boost::geometry::difference(output, polygon, tmp_union);
      output = tmp_union;
    }
  }
  if (boost::geometry::is_empty(output) ||
      ((BOOST_EPS > 0) && boost::geometry::area(output) < BOOST_EPS)) {
    res = true;
  } else {
    res = false;
  }

  return 0;
}

template int boost_polygon_enclosure_grid<ContainerGrid<HorizontalGrid>>(
    ContainerGrid<HorizontalGrid>& grid, std::vector<BoostPolygon*> polygons,
    AABB& aabb, const BoostPolygon* obj_poly, bool& res);
template int boost_polygon_enclosure_grid<ContainerGrid<VerticalGrid>>(
    ContainerGrid<VerticalGrid>& grid, std::vector<BoostPolygon*> polygons,
    AABB& aabb, const BoostPolygon* obj_poly, bool& res);

int boost_ccd_convex_hull_polygon_enclosure(const ShapeGroup& sg,
                                            const RectangleOBB& pos1,
                                            const RectangleOBB& pos2,
                                            bool& res) {
  std::vector<bg_polygon_type> poly_vec;

  int err = 0;

  collision::PolygonConstPtr resPoly;

  {
    // STACK_TIMER timer_chull(TIMER_boost_obb_obb_convex_hull);

    err = boost_obb_obb_convex_hull(pos1, pos2, resPoly);
  }
  if (err) return err;
  collision::RectangleAABBConstPtr aabb;

  {
    // STACK_TIMER timer_get_cand(TIMER_get_cand);
    aabb = resPoly->getAABB();
    const collision::RectangleAABB* aabb_ptr = aabb.get();
    err = boost_get_candidate_polygons_no_merge(sg, *aabb_ptr, poly_vec);
    if (err) return err;
  }

  bg_mpolygon_type output;
  bg_polygon_type cur_poly = *(BoostPolygon(resPoly.get()).getInternal());
  output.push_back(cur_poly);
  bg_mpolygon_type tmp_union;
  {
    // STACK_TIMER timer_difference(TIMER_difference);
    for (const auto &polygon : poly_vec) {
      boost::geometry::clear(tmp_union);
      boost::geometry::difference(output, polygon, tmp_union);
      output = tmp_union;
    }
  }
  if (boost::geometry::is_empty(output) ||
      ((BOOST_EPS > 0) && boost::geometry::area(output) < BOOST_EPS)) {
    res = true;
  } else {
    // std::cout << boost::geometry::area(output) << std::endl;
    res = false;
  }
  return 0;
}

int boost_ccd_obb_sum_polygon_enclosure(const ShapeGroup& sg,
                                        const RectangleOBB& pos1,
                                        const RectangleOBB& pos2, bool& res) {
  std::vector<bg_polygon_type> poly_vec;

  collision::RectangleOBBConstPtr resOBB =
      detail::geometry_queries::ccd_merge_entities(&pos1, &pos2);

  return boost_polygon_enclosure(sg, *resOBB, res);
}

}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision
