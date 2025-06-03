#pragma once

#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/solvers/boost/boost_object_internal.h"

#include <memory>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>

namespace collision {
namespace solvers {
namespace solverBoost {

typedef boost::geometry::model::d2::point_xy<double> bg_point_type;
typedef boost::geometry::model::polygon<bg_point_type> bg_polygon_type;

class BoostPolygon;

typedef std::shared_ptr<BoostPolygon> BoostPolygonPtr;
typedef std::shared_ptr<const BoostPolygon> BoostPolygonConstPtr;

/*!
        \brief BoostPolygon allows to use boost.Geometry functions for Polygon
   objects.

        The class is used internally to provide functionality for polygon
   operation queries.

*/

class BoostPolygon : public BoostObjectInternal {
 public:
  /*!
   \brief Constructs a BoostPolygon from Polygon.

   The input Polygon shall be counterclockwise-oriented and not closed. The
   first vertex is appended to the resulting polygon in the end so as to make it
   closed.

  */

  BoostPolygon(void) {}
  BoostPolygon(const BoostPolygon &other) {
    this->m_boost_polygon = other.m_boost_polygon;
#if ENABLE_POLYGON_VALIDITY_CHECKS
    boost::geometry::validity_failure_type failure;
    bool valid = boost::geometry::is_valid(m_boost_polygon, failure);
    if (!valid) {
      std::cout << "COLLISION_BOOST_POLYGON: the boost polygon is invalid"
                << std::endl;
    }
#endif
  }

  BoostPolygon(const collision::Polygon *obj) {
    auto vertices = obj->getVertices();
    for (auto vert = vertices.rbegin(); vert < vertices.rend(); vert++) {
      boost::geometry::append(m_boost_polygon,
                              bg_point_type((*vert)[0], (*vert)[1]));
    }
    auto vert = *(obj->getVertices().rbegin());
    boost::geometry::append(m_boost_polygon, bg_point_type(vert[0], vert[1]));

    int num_holes = 0;
    if ((num_holes = obj->getHoleVertices().size()) > 0) {
      boost::geometry::interior_rings(m_boost_polygon).resize(num_holes);
      int hole_ind = 0;
      for (const auto &hole_contour : obj->getHoleVertices()) {
        for (const auto &vert : hole_contour) {
          boost::geometry::append(m_boost_polygon,
                                  bg_point_type(vert[0], vert[1]), hole_ind);
        }
        auto vert = hole_contour.front();
        boost::geometry::append(m_boost_polygon,
                                bg_point_type(vert[0], vert[1]), hole_ind);
        hole_ind++;
      }
    }

#if ENABLE_POLYGON_VALIDITY_CHECKS
    boost::geometry::validity_failure_type failure;
    bool valid = boost::geometry::is_valid(m_boost_polygon, failure);
    if (!valid) {
      std::cout << "COLLISION_BOOST_POLYGON: the boost polygon is invalid"
                << std::endl;
    }
#endif
  }
  /*!
   \brief Constructs a BoostPolygon from RectangleOBB.
  */
  BoostPolygon(const collision::RectangleOBB *obj) {
    for (int i = 0; i < 4; i++) {
      double x = obj->segments()[i].point1().x;
      double y = obj->segments()[i].point1().y;
      boost::geometry::append(m_boost_polygon, bg_point_type(x, y));
    }
    double x = obj->segments()[0].point1().x;
    double y = obj->segments()[0].point1().y;
    boost::geometry::append(m_boost_polygon, bg_point_type(x, y));
#if ENABLE_POLYGON_VALIDITY_CHECKS
    boost::geometry::validity_failure_type failure;
    bool valid = boost::geometry::is_valid(m_boost_polygon, failure);
    if (!valid) {
      std::cout << "COLLISION_BOOST_POLYGON: the boost polygon is invalid"
                << std::endl;
    }
#endif
  }

  /*!
   \brief Constructs a BoostPolygon from RectangleAABB.
  */
  BoostPolygon(const collision::RectangleAABB *obj) {
    for (int i = 0; i < 4; i++) {
      double x = obj->segments()[i].point1().x;
      double y = obj->segments()[i].point1().y;
      boost::geometry::append(m_boost_polygon, bg_point_type(x, y));
    }
    double x = obj->segments()[0].point1().x;
    double y = obj->segments()[0].point1().y;
    boost::geometry::append(m_boost_polygon, bg_point_type(x, y));

#if ENABLE_POLYGON_VALIDITY_CHECKS
    bool valid = boost::geometry::is_valid(m_boost_polygon);
#endif
  }

  /*!
   \brief Constructs a BoostPolygon from Boost.Geometry polygon.
  */
  BoostPolygon(const bg_polygon_type polygon) {
#if ENABLE_POLYGON_VALIDITY_CHECKS
    boost::geometry::validity_failure_type failure;
    bool valid = boost::geometry::is_valid(polygon, failure);
    if (!valid) {
      std::cout
          << "COLLISION_BOOST_POLYGON: the constructed boost polygon is invalid"
          << std::endl;
    }
#endif

    m_boost_polygon = polygon;
  }

  virtual ~BoostPolygon(void) {}

  int extend(std::vector<BoostPolygon> &output, double radius) const;

  int buffer(std::vector<BoostPolygon> &output, double radius) const;

  BoostPolygonConstPtr convexHull(void) const;

  collision::PolygonConstPtr toPolygon(
      triangulation::TriangulationQuality qual =
          triangulation::TriangulationQuality()) const;

  int getTrapezoids(std::vector<BoostPolygon> &output, int axis = 0) const;

  /*!
        \brief Returns true if the BoostPolygon is completely contained within
     another BoostPolygon object.
  */
  bool within(const BoostPolygon &other) const {
    return boost::geometry::within(m_boost_polygon, other.m_boost_polygon);
  }
  /*!
        \brief Returns the contained boost.Geometry polygon object.
  */
  bg_polygon_type *getInternal(void) const { return &m_boost_polygon; }

 private:
  mutable bg_polygon_type m_boost_polygon;
};

}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision
