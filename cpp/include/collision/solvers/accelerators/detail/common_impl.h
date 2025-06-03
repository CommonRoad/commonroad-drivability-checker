

#ifndef CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_IMPL_H_
#define CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_IMPL_H_

namespace collision {
namespace detail {
namespace accelerators {
struct ObjectProxy {
  uint64_t mask;
  CollisionObject* obj_ptr;
  // collision::RectangleAABB* aabb;
};
}  // namespace accelerators
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_IMPL_H_ */
