#include "collision/solvers/accelerators/detail/container_box2d_inl.h"
#include "collision/solvers/accelerators/detail/container_fcl_inl.h"
#include "collision/solvers/accelerators/detail/container_grid_inl.h"

#include "collision/solvers/accelerators/detail/container_box2d.h"
#include "collision/solvers/accelerators/detail/container_fcl.h"
#include "collision/solvers/accelerators/detail/container_grid.h"

namespace collision {
namespace detail {
namespace accelerators {
template class ContainerGrid<class HorizontalGrid>;
template class ContainerGrid<class VerticalGrid>;

template <class DIRECTION>
int windowQuery(ContainerGrid<DIRECTION>& container, AABB& aabb,
                aligned_vector<int>& candidates) {
  return container.windowQuery(aabb, candidates);
}

template int windowQuery(ContainerGrid<class HorizontalGrid>& container,
                         AABB& aabb, aligned_vector<int>& candidates);
template int windowQuery(ContainerGrid<class VerticalGrid>& container,
                         AABB& aabb, aligned_vector<int>& candidates);

}  // namespace accelerators
}  // namespace detail
}  // namespace collision
