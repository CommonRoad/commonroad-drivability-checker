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

namespace detail {

class OBBDenseTrajectoryBatch {
 public:
  typedef collision::detail::OBB OBB;
  typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
  typedef std::vector<OBB, Eigen::aligned_allocator<OBB>>::const_iterator
      OBBConstIterator;

  int size(void) const { return traj_offsets_.size(); }

  int get_object_at_time_ptr(int traj_id, int timestep, OBB& obb,
                             AABB& aabb) const {
    if (!(traj_id >= 0 && traj_id < size())) {
      return -1;
    }
    int start_step = start_step_(traj_id);
    int end_step = start_step + traj_length_ - 1;

    if (!((timestep >= start_step) && (timestep <= end_step))) {
      return -1;
    }

    if (!(aabbs_valid_)) computeAABBs();

    int real_timestep = timestep - start_step;
    int offset = traj_offsets_[traj_id] + real_timestep;
    aabb = *(aabb_begin_ + offset);
    obb = *(obb_begin_ + offset);
    return 0;
  }

  void invalidateAABBs(void) { aabbs_valid_ = false; }

  // no input argument verification
  int get_start_step(int traj_id) const { return start_step_[traj_id]; }

  // no verification of trajectory length
  VectorXi get_start_step(void) const { return start_step_; }

  int length(void) const { return traj_length_; }

  // no checking if trajectory batch is empty
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OBBDenseTrajectoryBatch(
      std::vector<OBB, Eigen::aligned_allocator<OBB>> rectangles,
      int traj_length, const VectorXi& start_step)
      : rectangles_(rectangles),
        traj_length_(traj_length),
        start_step_(start_step) {
    int num_traj = rectangles.size() / traj_length;
    if (num_traj * traj_length != rectangles.size()) throw;

    num_traj_ = num_traj;

    traj_offsets_.resize(num_traj_);

    for (int cc1 = 0; cc1 < num_traj; cc1++) {
      traj_offsets_[cc1] = cc1 * traj_length;
    }

    aabb_begin_ = aabbs_.begin();
    obb_begin_ = rectangles_.begin();
    invalidateAABBs();
  }

  int computeAABBs(void) const {
    aabbs_.clear();
    aabbs_.reserve(rectangles_.size());
    for (int cc1 = 0; cc1 < rectangles_.size(); cc1++) {
      aabbs_.push_back(rectangles_[cc1].getAABB());
    }
    aabb_begin_ = aabbs_.begin();
    aabbs_valid_ = true;
    return 0;
  }

  int preprocess_inplace(void);

 private:
  std::vector<OBB, Eigen::aligned_allocator<OBB>> rectangles_;
  mutable std::vector<AABB, Eigen::aligned_allocator<AABB>> aabbs_;
  mutable std::vector<AABB, Eigen::aligned_allocator<AABB>>::iterator
      aabb_begin_;
  std::vector<OBB, Eigen::aligned_allocator<OBB>>::iterator obb_begin_;
  mutable bool aabbs_valid_;
  int num_traj_;
  std::vector<int> traj_offsets_;
  VectorXi start_step_;
  int traj_length_;
};

}  // namespace detail

}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_SOLVERS_TRAJECTORY_QUERIES_H_ */
