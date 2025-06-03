
#ifndef CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_H_
#define CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_H_

namespace collision {
namespace detail {
namespace accelerators {
struct ContainerSettings {
  ContainerSettings(void)
      : fcl_broadphase_type(0),
        num_cells(32),
        fcl_num_buckets(10000),
        optimize_triangles(true){};

  int num_cells;
  int fcl_broadphase_type;
  int fcl_num_buckets;
  bool optimize_triangles;
};

struct ContainerCollisionRequest {
  ContainerCollisionRequest(void)
      : enable_verification(false), test_obb_bbox(false){};

  bool enable_verification;
  bool test_obb_bbox;
};
}  // namespace accelerators
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_COMMON_H_ */
