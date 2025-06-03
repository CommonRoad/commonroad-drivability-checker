#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "collision/solvers/boost/boost_collision_object.h"
#include "collision/solvers/boost/boost_object_polygon.h"

namespace test {
int test_polygon() {
  using namespace collision::solvers::solverBoost;
  // test creation of collision polygon from boost polygon

  typedef boost::geometry::model::d2::point_xy<double> point_type;
  typedef boost::geometry::model::polygon<point_type> polygon_type;
  polygon_type poly;
  boost::geometry::read_wkt(
      "POLYGON((-57.0635 -3.58045, -57.8088 -4.75336, -56.7036 -7.65533, "
      "-56.4646 -9.08261, -55.219 -9.42072, -54.6047 -9.54237, -53.868 "
      "-9.22164, -52.4139 -8.58345, -49.4691 -6.94159, -49.2295 -6.87687, "
      "-47.1079 -6.18013, -45.9159 -5.91411, -43.7897 -5.89581, -42.285 "
      "-5.95883, -40.1693 -6.38999, -38.1374 -6.87424, -35.7062 -7.67065, "
      "-34.6193 -8.40713, -34.1769 -8.74957, -31.785 -9.31504, -30.574 "
      "-9.70903, -26.909 -10.5865, -24.9817 -11.268, -22.6134 -12.2833, "
      "-21.0927 -13.3427, -20.006 -13.7029, -16.8407 -15.2466, -13.2203 "
      "-16.816, -11.3922 -17.4133, -10.7577 -17.6366, -8.54581 -18.2736, "
      "-7.18058 -18.6037, -4.86031 -19.0997, -4.25834 -19.3046, -3.39545 "
      "-19.3122, -0.985195 -19.7047, -0.340967 -19.9409, 0.75209 -19.8968, "
      "3.30498 -20.3562, 6.82228 -20.6405, 10.4384 -20.5995, 13.8557 -20.2595, "
      "14.7523 -20.1371, 18.3468 -19.4284, 18.9915 -19.4505, 22.5377 -18.5053, "
      "23.2192 -18.4658, 26.4885 -17.3378, 27.2915 -17.2273, 30.5962 -15.8816, "
      "32.7278 -14.6911, 33.8878 -14.1893, 36.2752 -12.7482, 38.5306 -11.1893, "
      "40.8465 -9.28305, 41.1732 -9.05935, 43.372 -7.07535, 45.2436 -5.2073, "
      "46.9925 -3.21699, 48.616 -1.35438, 50.628 0.592424, 52.158 1.37445, "
      "53.1148 2.07603, 50.9152 5.07491, 50.3177 6.38521, 49.9412 7.3891, "
      "48.8311 9.29466, 47.7653 12.692, 45.6416 16.3693, 43.7106 17.9319, "
      "41.4998 17.9314, 37.4335 19.4761, 34.3477 18.4887, 32.1299 18.0676, "
      "29.2684 18.0666, 26.5269 15.0811, 19.7706 14.5304, 15.8916 12.9664, "
      "14.8124 12.5622, 12.783 10.0368, 12.4488 9.65713, 11.1008 7.47097, "
      "8.9187 5.4486, 5.63935 1.10272, -6.34225 4.45488, -8.60563 7.7494, "
      "-16.2454 11.2148, -23.399 18.8388, -25.8497 20.2781, -28.8931 22.7068, "
      "-31.4379 22.8942, -32.5636 21.9563, -35.2968 20.2469, -37.3132 19.0133, "
      "-37.9703 18.2828, -40.4433 15.553, -40.9543 14.1728, -42.8783 10.3486, "
      "-46.3297 7.28714, -48.959 5.88183, -52.1939 2.65934, -54.8196 "
      "-0.896786, -57.0635 -3.58045))",
      poly);

  boost::geometry::correct(poly);

  BoostPolygon bp(poly);

  collision::PolygonConstPtr poly1 = bp.toPolygon();

  // test creation of boost polygon from collision Polygon

  auto holeverts = poly1->getHoleVertices();
  auto verts = poly1->getVertices();
  auto trimesh = poly1->getTriangleMesh();
  collision::PolygonConstPtr poly_2 = collision::PolygonConstPtr(
      new collision::Polygon(verts, holeverts, trimesh));

  collision::solvers::solverBoost::SolverEntity_Boost* boost_ent;

  poly_2->getSolverEntity(boost_ent);

  if (boost_ent->getBoostEntityType() ==
      BOOST_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_BOOST_OBJECT) {
    BoostCollisionObject* boostObj =
        static_cast<BoostCollisionObject*>(boost_ent);
    auto bp_ent = boostObj->getCollisionObject_boost();
    BoostPolygonConstPtr bp3 =
        std::static_pointer_cast<const BoostPolygon>(bp_ent);
    polygon_type poly2 = *(bp3->getInternal());
    if (boost::geometry::equals(poly, poly2)) {
      auto bp_ch = bp3->convexHull();
      auto boost_ch = *(bp_ch->getInternal());
      polygon_type hull;
      boost::geometry::convex_hull(poly, hull);
      if (boost::geometry::equals(boost_ch, hull)) {
        collision::RectangleAABBConstPtr aabb1(
            new collision::RectangleAABB(1, 1));

        BoostPolygonConstPtr bp4 =
            std::make_shared<const BoostPolygon>(aabb1.get());
        if (bp4->within(*bp3)) {
          collision::RectangleOBBConstPtr obb1(
              new collision::RectangleOBB(1, 1, 0));
          BoostPolygonConstPtr bp5 =
              std::make_shared<const BoostPolygon>(obb1.get());
          if (bp5->within(*bp3)) {
            BoostPolygonConstPtr bp6 =
                std::make_shared<const BoostPolygon>(*(bp5->getInternal()));
            if (bp6->within(*bp3)) {
              if (aabb1->collide(*poly_2)) {
                return 0;
              } else
                return -7;

            } else
              return -6;
          } else
            return -5;
        } else {
          return -4;
        }
      } else
        return -3;
    } else
      return -2;
  } else
    return -1;

  return 0;
}
}  // namespace test
