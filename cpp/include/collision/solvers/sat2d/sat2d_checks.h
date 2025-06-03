#pragma once
#ifndef CPP_COLLISION_INCLUDE_COLLISION_PRIMITIVE_FAST_CHECKS_H_
#define CPP_COLLISION_INCLUDE_COLLISION_PRIMITIVE_FAST_CHECKS_H_

#include "collision/solvers/sat2d/obb_sat2d.h"
#include "collision/solvers/sat2d/triangle_sat2d.h"

namespace collision {
namespace detail {
namespace sat2dChecks {

/** Returns true if the intersection of the shapes is non-empty. */
bool overlaps(const Triangle_SAT2D& tri, const OBB_SAT2D& obb);

}  // namespace sat2dChecks
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_PRIMITIVE_FAST_CHECKS_H_ */
