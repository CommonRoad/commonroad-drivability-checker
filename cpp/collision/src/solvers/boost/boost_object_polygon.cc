#include <algorithm>

#include <boost/polygon/polygon.hpp>
#include "collision/solvers/boost/boost_object_polygon.h"

namespace collision {
namespace solvers {
namespace solverBoost {
/*!
 \brief Computes a convex hull of the given BoostPolygon
*/
BoostPolygonConstPtr BoostPolygon::convexHull(void) const {
  bg_polygon_type hull;
  boost::geometry::convex_hull(m_boost_polygon, hull);
  return std::make_shared<const BoostPolygon>(hull);
}

/*!
 \brief Buffers (enlargens) the polygon with a given radius.
*/
int BoostPolygon::buffer(std::vector<BoostPolygon> &output,
                         double radius) const {
  typedef double coordinate_type;
  const double buffer_distance = radius;
  const int points_per_circle = 5;
  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
      distance_strategy(buffer_distance);
  boost::geometry::strategy::buffer::join_round join_strategy(
      points_per_circle);
  boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
  boost::geometry::strategy::buffer::point_circle circle_strategy(
      points_per_circle);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::model::multi_polygon<bg_polygon_type> mpol;
  mpol.emplace_back(m_boost_polygon);
  boost::geometry::model::multi_polygon<bg_polygon_type> bufres;

  boost::geometry::buffer(mpol, bufres, distance_strategy, side_strategy,
                          join_strategy, end_strategy, circle_strategy);
  for (auto el : bufres) {
    output.push_back(BoostPolygon(el));
  }

  return 0;
}

int BoostPolygon::extend(std::vector<BoostPolygon> &output,
                         double radius) const {
  typedef double coordinate_type;
  const double buffer_distance = radius;
  const int points_per_circle = 5;
  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
      distance_strategy(buffer_distance);
  boost::geometry::strategy::buffer::join_miter join_strategy;
  boost::geometry::strategy::buffer::end_flat end_strategy;
  boost::geometry::strategy::buffer::point_circle circle_strategy(
      points_per_circle);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::model::multi_polygon<bg_polygon_type> mpol;
  mpol.emplace_back(m_boost_polygon);
  boost::geometry::model::multi_polygon<bg_polygon_type> bufres;

  boost::geometry::buffer(mpol, bufres, distance_strategy, side_strategy,
                          join_strategy, end_strategy, circle_strategy);
  for (auto el : bufres) {
    output.push_back(BoostPolygon(el));
  }

  return 0;
}

/*!
 \brief Converts the BoostPolygon into a collision Polygon
*/
collision::PolygonConstPtr BoostPolygon::toPolygon(
    triangulation::TriangulationQuality qual) const {
#if ENABLE_POLYGON_VALIDITY_CHECKS
  boost::geometry::validity_failure_type failure;
  bool valid = boost::geometry::is_valid(m_boost_polygon, failure);
  if (!valid) {
    std::cout << "COLLISION_BOOST_POLYGON: the boost polygon is invalid"
              << std::endl;
  }
#endif
  // must be a closed polygon
  std::vector<Eigen::Vector2d> vertices;
  for (auto it = boost::begin(boost::geometry::exterior_ring(m_boost_polygon));
       (it != boost::end(boost::geometry::exterior_ring(m_boost_polygon))) &&
       (it + 1 != boost::end(boost::geometry::exterior_ring(m_boost_polygon)));
       ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    vertices.emplace_back(x, y);
  }

  std::reverse(std::begin(vertices),
               std::end(vertices));  // produce a CCW polygon, not closed

  int num_holes = boost::geometry::interior_rings(m_boost_polygon).size();

  std::vector<std::vector<Eigen::Vector2d>> hole_vertices;

  for (int hole_ind = 0; hole_ind < num_holes; hole_ind++) {
    auto cur_ring = boost::geometry::interior_rings(m_boost_polygon)[hole_ind];
    std::vector<Eigen::Vector2d> vertices_hole;
    for (auto it = boost::begin(cur_ring);
         (it != boost::end(cur_ring)) && (it + 1 != boost::end(cur_ring));
         ++it) {
      double x = boost::geometry::get<0>(*it);
      double y = boost::geometry::get<1>(*it);
      vertices_hole.emplace_back(x, y);
    }
    hole_vertices.push_back(vertices_hole);
  }

  collision::PolygonConstPtr ret =
      std::make_shared<const collision::Polygon>(vertices, hole_vertices, 0, qual);

  return ret;
}

}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision
