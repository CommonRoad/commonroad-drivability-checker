#include "collision/solvers/boost/boost_collision_queries.h"
#include "collision/solvers/boost/boost_helpers.h"

#include "collision/solvers/accelerators/detail/container_box2d_inl.h"
#include "collision/solvers/accelerators/detail/container_fcl_inl.h"
#include "collision/solvers/accelerators/detail/container_grid_inl.h"

#include "collision/solvers/geometry_queries.h"

#include "collision/solvers/trajectory_queries.h"

namespace collision {

namespace detail {
using namespace accelerators;

int OBBDenseTrajectoryBatch::preprocess_inplace(void) {
  if (traj_length_ < 2) return 0;
  invalidateAABBs();
  for (int traj_ind = 0; traj_ind < num_traj_; traj_ind++) {
    auto read_iterator = rectangles_.begin() + traj_ind * traj_length_;
    auto write_iterator = read_iterator;
    for (int timestep = 0; timestep < traj_length_ - 1; timestep++) {
      auto obb_cur = *read_iterator;
      auto obb_next = *(read_iterator + 1);

      if (timestep == traj_length_ - 2) {
        *(write_iterator + 1) = *write_iterator =
            collision::detail::geometry_queries::merge_obbs(obb_cur, obb_next);
      } else {
        *write_iterator++ =
            collision::detail::geometry_queries::merge_obbs(obb_cur, obb_next);
        read_iterator++;
      }
    }
  }
  return 0;
}

namespace trajectory_queries {

inline void get_time_step_bounds(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    int& min_time_step, int& max_time_step) {
  min_time_step = traj_in[0]->time_start_idx();
  max_time_step = traj_in[0]->time_end_idx();

  for (auto el : traj_in) {
    min_time_step = std::min(min_time_step, el->time_start_idx());
    max_time_step = std::max(max_time_step, el->time_end_idx());
  }
}
inline void get_time_step_bounds(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_in,
    int& min_time_step_traj, int& max_time_step_traj, int& min_time_step_obst,
    int& max_time_step_obst) {
  get_time_step_bounds(traj_in, min_time_step_traj, max_time_step_traj);
  get_time_step_bounds(obsts_in, min_time_step_obst, max_time_step_obst);
  min_time_step_traj = std::max(min_time_step_traj, min_time_step_obst);
  max_time_step_traj = std::min(max_time_step_traj, max_time_step_obst);
}

inline void retrieve_shapes_from_trajectories(
    int time_step,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectory_list,
    aligned_vector<collision::CollisionObject*>& shapes_out) {
  for (auto el : trajectory_list) {
    collision::CollisionObject* obj_cur =
        const_cast<collision::CollisionObject*>(
            el->getObstacleAtTimePtr(time_step));
    if (obj_cur) {
      switch (obj_cur->getCollisionObjectClass()) {
        case collision::OBJ_CLASS_SHAPE:
          shapes_out.push_back(obj_cur);
          break;
        case collision::OBJ_CLASS_SHAPEGROUP:
          for (auto shape :
               (static_cast<collision::ShapeGroup*>(obj_cur))->unpack()) {
            shapes_out.push_back(const_cast<collision::Shape*>(shape.get()));
          }
          break;
        default:
          throw;
      }
    }
  }
}

template <class T>
inline int trajectories_collision_staticobst_helper(
    T& obstacles_container,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_in,
    ContainerCollisionRequest req, aligned_vector<int>& result) {
  if (obsts_in.size() + 2 != result.size()) return -1;

  std::fill(result.begin(), result.end(), -1);
  int min_time_step = -1, max_time_step = -1;
  get_time_step_bounds(obsts_in, min_time_step, max_time_step);
  aligned_vector<bool> done(obsts_in.size());

  for (int cur_time_step = min_time_step; cur_time_step <= max_time_step;
       cur_time_step++) {
    for (int cur_obst = 0; cur_obst < obsts_in.size(); cur_obst++) {
      if (!done[cur_obst]) {
        auto cur_obj = obsts_in[cur_obst]->getObstacleAtTimePtr(cur_time_step);
        if (cur_obj && obstacles_container.checkCollision(cur_obj, req))

        {
          done[cur_obst] = true;
          result[cur_obst] = cur_time_step;
        }
      }
    }
  }
  result[obsts_in.size()] = (obstacles_container.numcands());
  result[obsts_in.size() + 1] = (obstacles_container.numchecks());
  return 0;
}

int trajectories_collision_staticobst_grid(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const collision::ShapeGroup& obj,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  ContainerSettings sett;
  sett.num_cells = treq.num_cells;
  sett.optimize_triangles = treq.optimize_triangles;
  TIMER_START(TIMER_grid_build);
  collision::AABB bounds;
  int orientation = 0;
  getBestGridOrientationAndBounds(&obj, bounds, orientation);
  if (!treq.auto_orientation) orientation = 0;
  if (orientation == 0) {
    ContainerGrid<HorizontalGrid> obstacles_grid(&obj, &bounds, sett);

    TIMER_STOP(TIMER_grid_build);
    ContainerCollisionRequest creq;
    creq.enable_verification = treq.enable_verification;
    creq.test_obb_bbox = true;
    return trajectories_collision_staticobst_helper(obstacles_grid, traj_in,
                                                    creq, result);
  } else {
    bounds.swapAxes();
    TIMER_STOP(TIMER_grid_build);
    ContainerGrid<VerticalGrid> obstacles_grid(&obj, &bounds, sett);
    ContainerCollisionRequest creq;
    creq.enable_verification = treq.enable_verification;
    creq.test_obb_bbox = true;
    return trajectories_collision_staticobst_helper(obstacles_grid, traj_in,
                                                    creq, result);
  }
}

template <class T>
inline int trajectories_collision_dynamic_grid_helper(
    bool enable_verification, aligned_vector<bool>& done, T& obstacles_grid,
    int cur_time_step, int& numcands_global, int& numchecks_global,
    aligned_vector<int>& result,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_in) {
  for (int cur_obst = 0; cur_obst < obsts_in.size(); cur_obst++) {
    ContainerCollisionRequest creq;
    creq.enable_verification = enable_verification;
    creq.test_obb_bbox = false;
    if (!done[cur_obst]) {
      auto cur_obj = obsts_in[cur_obst]->getObstacleAtTimePtr(cur_time_step);
      if (cur_obj && obstacles_grid.checkCollision(cur_obj, creq)) {
        done[cur_obst] = true;
        result[cur_obst] = cur_time_step;
      }
    }
  }
  numcands_global += obstacles_grid.numcands();
  numchecks_global += obstacles_grid.numchecks();
  return 0;
}

int trajectories_collision_dynamic_grid(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_scenario,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  int numcol_cc = 0;
  int numcands_global = 0;
  int numchecks_global = 0;

  int min_time_step_traj = -1, max_time_step_traj = -1;
  int min_time_step_obst = -1, max_time_step_obst = -1;
  get_time_step_bounds(traj_in, obsts_scenario, min_time_step_traj,
                       max_time_step_traj, min_time_step_obst,
                       max_time_step_obst);

  aligned_vector<bool> done(traj_in.size());
  std::fill(result.begin(), result.end(), -1);
  aligned_vector<collision::CollisionObject*> col_objs_tvobst;

  for (int cur_time_step = min_time_step_traj;
       cur_time_step <= max_time_step_traj; cur_time_step++) {
    col_objs_tvobst.clear();

    retrieve_shapes_from_trajectories(cur_time_step, obsts_scenario,
                                      col_objs_tvobst);

    if (col_objs_tvobst.size() == 0) continue;

    TIMER_START(TIMER_grid_build);
    collision::AABB bounds;
    int orientation = 0;
    getBestGridOrientationAndBounds(col_objs_tvobst, bounds, orientation);

    if (orientation == 0) {
      ContainerSettings sett;
      sett.num_cells = treq.num_cells;
      ContainerGrid<HorizontalGrid> obstacles_grid(col_objs_tvobst, &bounds,
                                                   sett);

      TIMER_STOP(TIMER_grid_build);

      int err = trajectories_collision_dynamic_grid_helper(
          treq.enable_verification, done, obstacles_grid, cur_time_step,
          numcands_global, numchecks_global, result, traj_in);

    } else {
      ContainerSettings sett;
      sett.num_cells = treq.num_cells;
      bounds.swapAxes();
      ContainerGrid<VerticalGrid> obstacles_grid(col_objs_tvobst, &bounds,
                                                 sett);

      TIMER_STOP(TIMER_grid_build);

      int err = trajectories_collision_dynamic_grid_helper(
          treq.enable_verification, done, obstacles_grid, cur_time_step,
          numcands_global, numchecks_global, result, traj_in);
    }
  }
  result[traj_in.size()] = numcands_global;
  result[traj_in.size() + 1] = numchecks_global;

  return 0;
}

template <typename T>
inline int trajectories_collision_dynamic_helper(
    bool enable_verification, aligned_vector<int>& result,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_scenario,
    const ContainerSettings& sett) {
  int numcol_cc = 0;
  int numcands_global = 0;
  int numchecks_global = 0;

  int min_time_step_traj = -1, max_time_step_traj = -1;
  int min_time_step_obst = -1, max_time_step_obst = -1;
  get_time_step_bounds(obsts_in, obsts_scenario, min_time_step_traj,
                       max_time_step_traj, min_time_step_obst,
                       max_time_step_obst);

  aligned_vector<bool> done(obsts_in.size());
  std::fill(result.begin(), result.end(), -1);

  aligned_vector<collision::CollisionObject*> col_objs_tvobst;

  for (int cur_time_step = min_time_step_traj;
       cur_time_step <= max_time_step_traj; cur_time_step++) {
    col_objs_tvobst.clear();

    retrieve_shapes_from_trajectories(cur_time_step, obsts_scenario,
                                      col_objs_tvobst);

    TIMER_START(TIMER_grid_build);
    T obstacles_grid(col_objs_tvobst, sett);

    TIMER_STOP(TIMER_grid_build);

    ContainerCollisionRequest creq;
    creq.enable_verification = enable_verification;
    creq.test_obb_bbox = false;
    for (int cur_obst = 0; cur_obst < obsts_in.size(); cur_obst++) {
      if (!done[cur_obst]) {
        auto cur_obj = obsts_in[cur_obst]->getObstacleAtTimePtr(cur_time_step);
        if (cur_obj && obstacles_grid.checkCollision(cur_obj, creq)) {
          done[cur_obst] = true;
          result[cur_obst] = cur_time_step;
        }
      }
    }
    numcands_global += obstacles_grid.numcands();
    numchecks_global += obstacles_grid.numchecks();
  }

  result[obsts_in.size()] = numcands_global;
  result[obsts_in.size() + 1] = numchecks_global;

  return 0;
}

int trajectories_collision_dynamic_box2d(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_scenario,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  ContainerSettings sett;

  return trajectories_collision_dynamic_helper<ContainerBox2D>(
      treq.enable_verification, result, traj_in, obsts_scenario, sett);
}

int trajectories_collision_dynamic_fcl(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        obsts_scenario,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  ContainerSettings sett;
  sett.fcl_broadphase_type = treq.fcl_broadphase_type;
  sett.fcl_num_buckets = treq.fcl_num_buckets;
  sett.num_cells = treq.num_cells;

  return trajectories_collision_dynamic_helper<ContainerFCL>(
      treq.enable_verification, result, traj_in, obsts_scenario, sett);
}

int trajectories_collision_staticobst_fcl(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const collision::ShapeGroup& obj,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result)

{
  TIMER_START(TIMER_grid_build);
  ContainerSettings sett;
  sett.num_cells = treq.num_cells;
  sett.fcl_num_buckets = treq.fcl_num_buckets;
  sett.fcl_broadphase_type = treq.fcl_broadphase_type;
  ContainerFCL obstacles_cont(&obj, sett);

  TIMER_STOP(TIMER_grid_build);

  ContainerCollisionRequest creq;
  creq.enable_verification = treq.enable_verification;
  creq.test_obb_bbox = false;

  return trajectories_collision_staticobst_helper(obstacles_cont, traj_in, creq,
                                                  result);
}

int trajectories_enclosure_polygons_static_grid(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const collision::ShapeGroup& obj,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  int min_time_step = -1, max_time_step = -1;
  get_time_step_bounds(traj_in, min_time_step, max_time_step);
  aligned_vector<bool> done(traj_in.size());

  std::fill(result.begin(), result.end(), -1);
  TIMER_START(TIMER_grid_build);
  collision::AABB bounds;
  int orientation = 0;

  getBestGridOrientationAndBounds(&obj, bounds, orientation);
  ContainerSettings sett;
  sett.num_cells = treq.num_cells;
  ContainerGrid<HorizontalGrid> obstacles_grid(&obj, &bounds, sett);

  std::vector<collision::BoostPolygon*> polygons_vec;

  for (auto poly : obj.unpack()) {
    auto poly_obj = poly.get();
    if (poly_obj->getCollisionObjectType() == collision::OBJ_TYPE_POLYGON) {
      polygons_vec.push_back(static_cast<collision::BoostPolygon*>(
          collision::solvers::solverBoost::get_boost_object_ptr(poly_obj)
              ->getCollisionObject_boost()
              .get()));
    } else {
      throw std::invalid_argument(
          "the shape group for enclosure checks can contain only "
          "collision::Polygon objects");
    }
  }

  TIMER_STOP(TIMER_grid_build);

  for (int cur_time_step = min_time_step; cur_time_step <= max_time_step;
       cur_time_step++) {
    for (int cur_obst = 0; cur_obst < traj_in.size(); cur_obst++) {
      if (!done[cur_obst]) {
        auto cur_obj = traj_in[cur_obst]->getObstacleAtTimePtr(cur_time_step);

        if (cur_obj) {
          collision::BoostPolygon bp1;
          switch (cur_obj->getCollisionObjectType()) {
            case collision::OBJ_TYPE_OBB_BOX:

            {
              TIMER_START(TIMER_poly_build);
              bp1 = collision::BoostPolygon(
                  static_cast<const collision::RectangleOBB*>(cur_obj));
              TIMER_STOP(TIMER_poly_build);
            } break;
            case collision::OBJ_TYPE_AABB_BOX:

            {
              TIMER_START(TIMER_poly_build);
              bp1 = collision::BoostPolygon(
                  static_cast<const collision::RectangleAABB*>(cur_obj));
              TIMER_STOP(TIMER_poly_build);
            } break;
            default:
              throw std::invalid_argument(
                  "the trajectory for polygon enclosure checks can consist "
                  "only of OBB or AABB boxes");
              break;
          }

          collision::AABB aabb = getAABB(cur_obj);

          bool bRes = false;
          boost_polygon_enclosure_grid(obstacles_grid, polygons_vec, aabb,
                                       (collision::BoostPolygon*)&bp1, bRes);

          if (!bRes) {
            done[cur_obst] = true;
            result[cur_obst] = cur_time_step;
          }
        }
      }
    }
  }
  result[traj_in.size()] = obstacles_grid.numcands();
  result[traj_in.size() + 1] = obstacles_grid.numchecks();
  return 0;
}

int trajectories_collision_staticobst_box2d(
    const aligned_vector<const collision::TimeVariantCollisionObject*>& traj_in,
    const collision::ShapeGroup& obj,
    const TrajectoryRequestCollisionTime& treq, aligned_vector<int>& result) {
  TIMER_START(TIMER_grid_build);
  ContainerSettings sett;
  ContainerBox2D obstacles_cont(&obj, sett);

  TIMER_STOP(TIMER_grid_build);

  ContainerCollisionRequest creq;
  creq.enable_verification = treq.enable_verification;
  creq.test_obb_bbox = false;

  return trajectories_collision_staticobst_helper(obstacles_cont, traj_in, creq,
                                                  result);
}
}  // namespace trajectory_queries
}  // namespace detail
namespace trajectory_queries {
using namespace detail::trajectory_queries;
int trajectories_collision_time_dynamic_obstacles(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        dynamic_obstacles,
    TrajectoryQueryResult& res, const TrajectoryRequest& treq) {
  auto request =
      static_cast<const collision::TrajectoryRequestCollisionTime&>(treq);
  aligned_vector<int>* result =
      static_cast<collision::TrajectoryQueryResultCollisionTime&>(res).result;
  if (!result) return -1;
  switch (request.broadphase_class) {
    case BC_UNIFORMGRID:
      return trajectories_collision_dynamic_grid(
          trajectories_in, dynamic_obstacles, request, *result);
      break;
    case BC_FCL:
      return trajectories_collision_dynamic_fcl(
          trajectories_in, dynamic_obstacles, request, *result);
      break;
    case BC_BOX2D:
      return trajectories_collision_dynamic_box2d(
          trajectories_in, dynamic_obstacles, request, *result);
      break;
    default:
      throw;
  }
  return -1;
}

int trajectories_collision_time_static_obstacles(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const collision::ShapeGroup& static_obstacles, TrajectoryQueryResult& res,
    const TrajectoryRequest& treq) {
  auto request =
      static_cast<const collision::TrajectoryRequestCollisionTime&>(treq);
  aligned_vector<int>* result =
      static_cast<collision::TrajectoryQueryResultCollisionTime&>(res).result;
  if (!result) return -1;

  switch (request.broadphase_class) {
    case BC_UNIFORMGRID:
      return trajectories_collision_staticobst_grid(
          trajectories_in, static_obstacles, request, *result);
      break;
    case BC_FCL:
      return trajectories_collision_staticobst_fcl(
          trajectories_in, static_obstacles, request, *result);
      break;
    case BC_BOX2D:
      return trajectories_collision_staticobst_box2d(
          trajectories_in, static_obstacles, request, *result);
      break;
    default:
      throw;
  }
  return -1;
}
int trajectories_enclosure_time_polygons_static(
    const aligned_vector<const collision::TimeVariantCollisionObject*>&
        trajectories_in,
    const collision::ShapeGroup& polygons, TrajectoryQueryResult& res,
    const TrajectoryRequest& treq) {
  auto request =
      static_cast<const collision::TrajectoryRequestCollisionTime&>(treq);
  aligned_vector<int>* result =
      static_cast<collision::TrajectoryQueryResultCollisionTime&>(res).result;
  if (!result) return -1;

  switch (request.broadphase_class) {
    case BC_UNIFORMGRID:
      return trajectories_enclosure_polygons_static_grid(
          trajectories_in, polygons, request, *result);
      break;
    default:
      throw;
  }
  return -1;
}

/*!
 \brief Preprocesses a TimeVariantCollisionObject for continuous collision
 detection using OBB sum hull for the consecutive pairs of trajectory OBB boxes

 \param[out] reference to the new object to be created

*/

int trajectory_preprocess_obb_sum(const TimeVariantCollisionObject* obj_old,
                                  TimeVariantCollisionObjectPtr& obj_new) {
  TimeVariantCollisionObjectPtr new_obj = TimeVariantCollisionObjectPtr(
      new TimeVariantCollisionObject(obj_old->time_start_idx()));

  obj_new = new_obj;

  auto new_obj_ptr = obj_new.get();

  auto time_start_idx_ = obj_old->time_start_idx();
  auto time_end_idx_ = obj_old->time_end_idx();

  if (time_end_idx_ - time_start_idx_ <= 0) return 0;
  for (int i = time_start_idx_; i <= time_end_idx_; i++) {
    CollisionObjectConstPtr first;
    CollisionObjectConstPtr second;
    if (i == time_end_idx_) {
      first = obj_old->getObstacleAtTime(i - 1);
      second = obj_old->getObstacleAtTime(i);
    } else {
      first = obj_old->getObstacleAtTime(i);
      second = obj_old->getObstacleAtTime(i + 1);
    }

    if (!first || !second) {
      return -1;
    }

    CollisionObjectConstPtr element_new;
    element_new =
        geometry_queries::ccd_merge_entities(first.get(), second.get());
    if (!element_new) {
      return -1;
    } else {
      new_obj->appendObstacle(element_new);
    }
  }
  obj_new = new_obj;
  return 0;
}

namespace deprecated {

/*!
      \brief Checks if the convex hull of the Boolean union of two given OBB
   boxes collides with any of the ShapeGroup objects.

       Returns 0 if no error has occured.

       \param[in] sg ShapeGroup with static obstacles

       \param[in] pos1 OBB rectangle 1

       \param[in] pos2 OBB rectangle 2

       \param[out] res Boolean collision result

*/

std::size_t ccd_trajectory_test_collision_chull(const ShapeGroup& sg,
                                                const RectangleOBB& pos1,
                                                const RectangleOBB& pos2,
                                                bool& res) {
  if (boost_ccd_convex_hull_collision(sg, pos1, pos2, res))
    return 1;
  else
    return 0;
}

/*!
      \brief Checks if the over-approximative OBB box (OBB sum hull) for the two
   given OBB boxes collides with any of the ShapeGroup objects.

       Returns 0 if no error has occured.

       \param[in] sg ShapeGroup with static obstacles

       \param[in] pos1 OBB rectangle 1

       \param[in] pos2 OBB rectangle 2

       \param[out] res Boolean collision result

*/
std::size_t ccd_trajectory_test_collision_obb_sum(const ShapeGroup& sg,
                                                  const RectangleOBB& pos1,
                                                  const RectangleOBB& pos2,
                                                  bool& res) {
  if (boost_ccd_obb_sum_collision(sg, pos1, pos2, res))
    return 1;
  else
    return 0;
}

/*!
      \brief Checks if the convex hull of the Boolean union of two given OBB
   boxes is enclosed within the ShapeGroup polygons.

       Returns 0 if no error has occured.

       \param[in] sg ShapeGroup with static obstacles

       \param[in] pos1 OBB rectangle 1

       \param[in] pos2 OBB rectangle 2

       \param[out] res Boolean collision result

*/

std::size_t ccd_trajectory_test_polygon_enclosure_chull(
    const ShapeGroup& sg, const RectangleOBB& pos1, const RectangleOBB& pos2,
    bool& res) {
  if (boost_ccd_convex_hull_polygon_enclosure(sg, pos1, pos2, res))
    return 1;
  else
    return 0;
}

/*!
      \brief Checks if the over-approximative OBB box (OBB sum hull) for the two
   given OBB boxes boxes is enclosed within the ShapeGroup polygons.

       Returns 0 if no error has occured.

       \param[in] sg ShapeGroup with static obstacles

       \param[in] pos1 OBB rectangle 1

       \param[in] pos2 OBB rectangle 2

       \param[out] res Boolean collision result

*/
std::size_t ccd_trajectory_test_polygon_enclosure_obb_sum(
    const ShapeGroup& sg, const RectangleOBB& pos1, const RectangleOBB& pos2,
    bool& res) {
  if (boost_ccd_obb_sum_polygon_enclosure(sg, pos1, pos2, res))
    return 1;
  else
    return 0;
}

}  // namespace deprecated

}  // namespace trajectory_queries
}  // namespace collision
