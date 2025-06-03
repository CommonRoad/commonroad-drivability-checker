#ifndef CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_CONTAINER_GRID_H_
#define CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_CONTAINER_GRID_H_

#include "collision/narrowphase/detail/aabb.h"
#include "collision/solvers/accelerators/declarations.h"
#include "collision/solvers/accelerators/detail/container_grid_common.h"

namespace collision {
namespace detail {
namespace accelerators {

template <class DIRECTION>
class ContainerGrid;

template <class DIRECTION>
int windowQuery(ContainerGrid<DIRECTION>& container, AABB& aabb,
                aligned_vector<int>& candidates);

}  // namespace accelerators
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_CONTAINER_GRID_H_ */
