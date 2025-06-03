#ifndef CPP_COLLISION_FCL_FCL_COLLISION_CHECKER_H_
#define CPP_COLLISION_FCL_FCL_COLLISION_CHECKER_H_

#include "collision/application.h"

#include <sstream>
#include <vector>

#include "collision/primitive_collision_checker.h"

#include "collision/collision_object.h"

#include "collision/narrowphase/rectangle_aabb.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include "collision/solvers/fcl/fcl_collision_object_group.h"
#include "collision/solvers/fcl/fcl_collision_requests.h"
#include "collision/solvers/fcl/fcl_decl.h"

#include "collision/i_collision_checker.h"

namespace collision {
class CollisionChecker;

namespace solvers {
namespace solverFCL {
class FCLCollisionChecker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FCLCollisionChecker(ICollisionChecker &cc);

  void addCollisionObject(CollisionObjectConstPtr co);
  void windowQuery_helper(const RectangleAABB &aabb,
                          ICollisionChecker &ret) const;
  bool collideHelper(CollisionObjectConstPtr co, int *collision_time,
                     std::vector<CollisionObjectConstPtr> *obstacles,
                     int max_obstacles = -1, bool ungroup_shape_groups = false,
                     bool ungroup_TV_obstacles = false) const;

 private:
  ICollisionChecker &m_cc;

  enum CollisionRequestType {
    collisionRequestBoolean = 0,
    collisionRequestListOfObstacles = 1,
    collisionRequestWindowQuery = 2

  };

  void invalidateParentMap(void);
  void setUpParentMap(void) const;

  void invalidateStaticObstaclesManager(void);
  void setUpStaticObstaclesManager(void) const;

  int set_up_dynamic_obstacles(int simulation_cur_time_idx_) const;
  int register_dynamic_obstacle(const FCLCollisionObjectGroup *obj) const;
  int register_dynamic_obstacle(const FCLCollisionObject *obj) const;

  int addCollisionObject_fcl(const FCLCollisionObject *co);
  int addCollisionObject_fcl(const FCLCollisionObjectGroup *co);
  void addCollisionObject_fcl(const TimeVariantCollisionObject *tvobj);

  bool collide_fcl(const FCLCollisionObject *col_obj_ptr,
                   int &time_of_collision,
                   CollisionRequestDataMultipleObstacles &reqData) const;
  bool collide_fcl(const FCLCollisionObjectGroup *col_obj_group,
                   int &time_of_collision,
                   CollisionRequestDataMultipleObstacles &reqData) const;

  bool collide_fcl(const FCLCollisionObjectGroup *,
                   int &time_of_collision) const;
  bool collide_fcl(const FCLCollisionObject *col_obj_ptr,
                   int &time_of_collision) const;

  bool collide_fcl(const TimeVariantCollisionObject *tvobj,
                   int &time_of_collision) const;
  bool collide_fcl(const TimeVariantCollisionObject *tvobj,
                   int &time_of_collision,
                   CollisionRequestDataMultipleObstacles &reqData) const;

  template <class T>
  bool collide_fcl_helper_simulate_static_subject(
      T col_entity, int &time_of_collision,
      CollisionRequestType reqType = collisionRequestBoolean,
      CollisionRequestData *reqData = 0) const;
  bool collide_fcl_helper_sim_TV_subject(
      const TimeVariantCollisionObject &tvobj, int &time_of_collision,
      CollisionRequestType reqType, CollisionRequestData *reqData) const;
  template <class T>
  bool collide_fcl_helper_dynamic_obstaclesEx(
      T col_entity, int sim_cur_time_idx, bool &result,
      CollisionRequestType reqType, CollisionRequestData *reqData) const;
  template <class T>
  bool collide_fcl_helper_dynamic_obstacles(T col_entity,
                                            int sim_cur_time_idx) const;
  template <class T>
  bool collide_fcl_helper_static_obstaclesEx(
      T col_entity, bool &result, CollisionRequestType reqType,
      CollisionRequestData *reqData) const;

  template <class T>
  bool collide_fcl_helper_static_obstacles(T col_entity) const;

 private:
  mutable std::unordered_map<const CollisionObject *,
                             std::list<CollisionObjectConstPtr>>
      parent_map;
  mutable fcl::DynamicAABBTreeCollisionManager<FCL_PRECISION> static_obstacles_;
  mutable fcl::DynamicAABBTreeCollisionManager<FCL_PRECISION>
      dynamic_obstacles_;
  std::vector<CollisionObjectConstPtr> tv_objects_;
  mutable int simulation_time_start_idx_;
  mutable int simulation_time_end_idx_;
  mutable int simulation_dynamic_;
  mutable bool static_obstacles_manager_valid;
  mutable bool parent_map_valid;

  mutable bool windowQueryDone;
};
}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision

#endif /* CPP_COLLISION_FCL_FCL_COLLISION_CHECKER_H_ */
