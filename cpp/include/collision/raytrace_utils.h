#ifndef CPP_COLLISION_RAYTRACE_UTILS_H_
#define CPP_COLLISION_RAYTRACE_UTILS_H_

#include <Eigen/Dense>
#include "collision/line_segment.h"
#include "collision/narrowphase/point.h"

namespace collision {
namespace raytrace {

bool rayTracePrimitive(
    std::vector<collision::CollisionObjectConstPtr> collision_objects,
    const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
    std::vector<LineSegment> &intersect, bool remove_overlaps);
bool rayTracePostprocess(const Eigen::Vector2d &point1,
                         const Eigen::Vector2d &point2,
                         std::vector<Eigen::Vector2d> inters1,
                         std::vector<LineSegment> &intersect,
                         const collision::CollisionObject *obj);
int rayTraceRemoveOverlaps(const std::vector<LineSegment>& intersect,
                           std::vector<LineSegment> &out_vec, int axis = 0);

}  // namespace raytrace
}  // namespace collision

#endif /* CPP_COLLISION_RAYTRACE_UTILS_H_ */
