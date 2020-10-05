#pragma once

#include "collision/shape_group.h"
#include "collision/solvers/boost/boost_object_polygon.h"

namespace collision {
namespace solvers {
namespace solverBoost {

collision::ShapeGroupPtr lane_polygons_postprocess(
    collision::ShapeGroup &sg_polys, double buf_width, bool triangulate);
}
}  // namespace solvers
}  // namespace collision
