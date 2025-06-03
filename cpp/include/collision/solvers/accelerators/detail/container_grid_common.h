#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_ACCELERATORS_DETAIL_CONTAINER_GRID_COMMON_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_ACCELERATORS_DETAIL_CONTAINER_GRID_COMMON_H_
namespace collision {
namespace detail {
namespace accelerators {

class DirectionType {};
class VerticalGrid : public DirectionType {};
class HorizontalGrid : public DirectionType {};
}  // namespace accelerators
}  // namespace detail
}  // namespace collision
#endif
