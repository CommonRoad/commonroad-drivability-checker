#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_QUERIES_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_QUERIES_H_

#include "collision/solvers/boost/boost_object_polygon.h"

namespace collision {
namespace solvers {
namespace solverBoost {

bool boost_within(const BoostPolygon& polyg1, const BoostPolygon& polyg2);

int boost_poly_poly_convex_hull(const Polygon& pos1, const Polygon& pos2,
                                collision::PolygonConstPtr& resPoly);

int boost_ccd_convex_hull_collision(const ShapeGroup& sg,
                                    const RectangleOBB& pos1,
                                    const RectangleOBB& pos2, bool& res);

int boost_ccd_obb_sum_collision(const ShapeGroup& sg, const RectangleOBB& pos1,
                                const RectangleOBB& pos2, bool& res);

int boost_polygon_enclosure(const ShapeGroup& sg, const RectangleOBB& obb,
                            bool& res);

template <typename T>
int boost_polygon_enclosure_grid(T& grid, std::vector<BoostPolygon*> polygons,
                                 AABB& aabb, const BoostPolygon* obj_poly,
                                 bool& res);

int boost_ccd_convex_hull_polygon_enclosure(const ShapeGroup& sg,
                                            const RectangleOBB& pos1,
                                            const RectangleOBB& pos2,
                                            bool& res);
int boost_ccd_obb_sum_polygon_enclosure(const ShapeGroup& sg,
                                        const RectangleOBB& pos1,
                                        const RectangleOBB& pos2, bool& res);

}  // namespace solverBoost

}  // namespace solvers

}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_BOOST_BOOST_COLLISION_QUERIES_H_ \
        */
