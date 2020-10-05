#ifndef CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_TRAJECTORY_QUERIES_H_
#define CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_TRAJECTORY_QUERIES_H_

#include "collision/solvers/accelerators/declarations.h"
#include "collision/time_variant_collision_object.h"

namespace collision {

enum TrajectoryRequestBroadphaseClass {
  BC_UNIFORMGRID = 0,
  BC_FCL,
  BC_BOX2D,
};

struct TrajectoryRequest {};

struct TrajectoryRequestCollisionTime : public TrajectoryRequest {
  TrajectoryRequestCollisionTime(void)
      : enable_verification(false),
        broadphase_class(BC_UNIFORMGRID),
        num_cells(32),
        fcl_broadphase_type(0),
        fcl_num_buckets(1000),
        auto_orientation(true),
        optimize_triangles(true){};
  TrajectoryRequestBroadphaseClass broadphase_class;
  bool enable_verification;
  int num_cells;
  int fcl_broadphase_type;
  int fcl_num_buckets;
  bool optimize_triangles;
  bool auto_orientation;
};

struct TrajectoryQueryResult {};

struct TrajectoryQueryResultCollisionTime : public TrajectoryQueryResult {
  TrajectoryQueryResultCollisionTime(aligned_vector<int>* res) : result(res){};
  aligned_vector<int>* result;
};

namespace trajectory_queries {

int trajectories_collision_time_dynamic_obstacles(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        dynamic_obstacles,
    TrajectoryQueryResult& res, const TrajectoryRequest& treq);
int trajectories_collision_time_static_obstacles(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const collision::ShapeGroup& static_obstacles, TrajectoryQueryResult& res,
    const TrajectoryRequest& treq);
int trajectories_enclosure_time_polygons_static(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const collision::ShapeGroup& polygons, TrajectoryQueryResult& res,
    const TrajectoryRequest& treq);

int trajectory_preprocess_obb_sum(const TimeVariantCollisionObject* obj_old,
                                  TimeVariantCollisionObjectPtr& obj_new);

namespace deprecated {  // slow deprecated functions for trajectory collision
                        // and polygon enclosure checks

std::size_t ccd_trajectory_test_collision_chull(const ShapeGroup& sg,
                                                const RectangleOBB& pos1,
                                                const RectangleOBB& pos2,
                                                bool& res);

std::size_t ccd_trajectory_test_collision_obb_sum(const ShapeGroup& sg,
                                                  const RectangleOBB& pos1,
                                                  const RectangleOBB& pos2,
                                                  bool& res);

std::size_t ccd_trajectory_test_polygon_enclosure_chull(
    const ShapeGroup& sg, const RectangleOBB& pos1, const RectangleOBB& pos2,
    bool& res);

std::size_t ccd_trajectory_test_polygon_enclosure_obb_sum(
    const ShapeGroup& sg, const RectangleOBB& pos1, const RectangleOBB& pos2,
    bool& res);
}  // namespace deprecated

}  // namespace trajectory_queries

}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_TRAJECTORY_QUERIES_H_ */
