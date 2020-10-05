#include "collision/solvers/fcl/fcl_collision_checker.h"
#include "collision/solvers/fcl/fcl_helpers.h"
#include "collision/time_variant_collision_object.h"

namespace collision {
namespace solvers {
namespace solverFCL {
FCLCollisionChecker::FCLCollisionChecker(ICollisionChecker &cc) : m_cc(cc) {
  simulation_time_start_idx_ = simulation_time_end_idx_ = -1;
  simulation_dynamic_ = 0;
  invalidateStaticObstaclesManager();
  invalidateParentMap();
  windowQueryDone = false;
}

void FCLCollisionChecker::invalidateParentMap(void) {
  parent_map_valid = false;
}

void FCLCollisionChecker::setUpParentMap(void) const {
  if (!parent_map_valid) {
    parent_map.clear();
    std::vector<CollisionObjectConstPtr> obst = m_cc.getObstacles();

    for (auto &obj : obst) {
      obj->addParentMap(parent_map);
    }
    parent_map_valid = true;
  }
}

void FCLCollisionChecker::addCollisionObject(CollisionObjectConstPtr co) {
  SolverEntity_FCL *obj_entity;
  FCL_COLLISION_ENTITY_TYPE enttype =
      get_object_fcl_entity_type(co, obj_entity);
  // const TimeVariantCollisionObject* tvobj;

  invalidateParentMap();
  invalidateStaticObstaclesManager();

  switch (enttype) {
    case COLLISION_ENTITY_TYPE_FCL_OBJECT:

      addCollisionObject_fcl(
          static_cast<const FCLCollisionObject *>(obj_entity));
      return;
      break;
    case COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP:

      addCollisionObject_fcl(
          static_cast<const FCLCollisionObjectGroup *>(obj_entity));
      return;
      break;
    case FCL_COLLISION_ENTITY_TYPE::FCL_COLLISION_ENTITY_TYPE_TVOBJECT:

      addCollisionObject_fcl(
          static_cast<const TimeVariantCollisionObject *>(co.get()));
      return;
      break;
    default:
      throw;
      return;
      break;
  }
}

int FCLCollisionChecker::addCollisionObject_fcl(
    const FCLCollisionObjectGroup *co) {
  // TODO: thread-safety: store all object instances locally?
  if (!co) return -1;
  std::vector<fcl::CollisionObject<FCL_PRECISION> *> vect;
  const IFCLCollisionObjectGroup *parent_interface = co->getParentInterface();
  if (!parent_interface) return -1;
  parent_interface->getCollisionObjects(vect);
  static_obstacles_.registerObjects(vect);
  return 0;
}

int FCLCollisionChecker::addCollisionObject_fcl(const FCLCollisionObject *co) {
  // TODO: thread-safety: store all object instances locally?
  if (!co) return -1;
  std::shared_ptr<fcl::CollisionObject<FCL_PRECISION>> col_obj =
      co->getCollisionObject_fcl();
  static_obstacles_.registerObject(col_obj.get());
  return 0;
}

void FCLCollisionChecker::addCollisionObject_fcl(
    const TimeVariantCollisionObject *tvobj) {
  // collision_objects_.push_back(tvobj->shared_from_this());

  tv_objects_.push_back(tvobj->shared_from_this());
  if (!simulation_dynamic_) {
    simulation_time_start_idx_ = tvobj->time_start_idx();
    simulation_time_end_idx_ = tvobj->time_end_idx();
    simulation_dynamic_ = 1;
  } else {
    if (simulation_time_start_idx_ > tvobj->time_start_idx())
      simulation_time_start_idx_ = tvobj->time_start_idx();
    if (simulation_time_end_idx_ < tvobj->time_end_idx())
      simulation_time_end_idx_ = tvobj->time_end_idx();
  }
  // TODO: thread-safety: store all object instances locally?
}

void FCLCollisionChecker::invalidateStaticObstaclesManager() {
  static_obstacles_manager_valid = false;
}

void FCLCollisionChecker::setUpStaticObstaclesManager() const {
  if (!static_obstacles_manager_valid) {
    static_obstacles_manager_valid = true;
    static_obstacles_.setup();
  }
}

int FCLCollisionChecker::set_up_dynamic_obstacles(
    int simulation_cur_time_idx_) const {
  if (!simulation_dynamic_ ||
      simulation_cur_time_idx_ < simulation_time_start_idx_ ||
      simulation_cur_time_idx_ > simulation_time_end_idx_)
    return -1;
  dynamic_obstacles_.clear();
  for (auto &c : tv_objects_) {
    const TimeVariantCollisionObject *tvobj =
        static_cast<const TimeVariantCollisionObject *>(c.get());
    if (!tvobj) {
      throw;
    }

    CollisionObjectConstPtr obj =
        tvobj->getObstacleAtTime(simulation_cur_time_idx_);
    if (obj) {
      SolverEntity_FCL *obj_entity;
      FCL_COLLISION_ENTITY_TYPE enttype =
          get_object_fcl_entity_type(obj, obj_entity);
      if (enttype == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
        const FCLCollisionObject *fclco =
            static_cast<const FCLCollisionObject *>(obj_entity);
        // TODO: only update the manager
        register_dynamic_obstacle(fclco);
      } else if (enttype == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
        const FCLCollisionObjectGroup *fclco_gr =
            static_cast<const FCLCollisionObjectGroup *>(obj_entity);
        register_dynamic_obstacle(fclco_gr);
      }
    }
  }
  dynamic_obstacles_.setup();
  return 0;
}

int FCLCollisionChecker::register_dynamic_obstacle(
    const FCLCollisionObjectGroup *obj) const {
  if (!obj) return -1;

  std::vector<fcl::CollisionObject<FCL_PRECISION> *> vect;
  const IFCLCollisionObjectGroup *parent_interface = obj->getParentInterface();
  if (!parent_interface) return -1;
  parent_interface->getCollisionObjects(vect);
  dynamic_obstacles_.registerObjects(vect);
  return 0;
}

int FCLCollisionChecker::register_dynamic_obstacle(
    const FCLCollisionObject *obj) const {
  if (!obj) return -1;
  dynamic_obstacles_.registerObject(obj->getCollisionObject_fcl().get());
  return 0;
}

bool FCLCollisionChecker::collideHelper(
    CollisionObjectConstPtr co, int *collision_time,
    std::vector<CollisionObjectConstPtr> *obstacles, int max_obstacles,
    bool ungroup_shape_groups, bool ungroup_TV_obstacles) const {
  const FCLCollisionObject *fclco;
  const FCLCollisionObjectGroup *fclgr;
  const TimeVariantCollisionObject *tvobj;

  setUpStaticObstaclesManager();
  SolverEntity_FCL *obj_entity;
  FCL_COLLISION_ENTITY_TYPE enttype =
      get_object_fcl_entity_type(co, obj_entity);

  int col_time = -1;
  bool res = false;
  switch (enttype) {
    case COLLISION_ENTITY_TYPE_FCL_OBJECT:
      fclco = static_cast<const FCLCollisionObject *>(obj_entity);
      if (obstacles) {
        CollisionRequestDataMultipleObstacles reqData(
            *obstacles, this->parent_map, max_obstacles);
        reqData.setUngroupShapeGroups(ungroup_shape_groups);
        reqData.setUngroupTvObstacles(ungroup_TV_obstacles);
        res = collide_fcl(fclco, col_time, reqData);
      } else {
        res = collide_fcl(fclco, col_time);
      }

      if (collision_time) {
        *collision_time = col_time;
      }
      return res;
      break;
    case COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP:
      fclgr = static_cast<const FCLCollisionObjectGroup *>(obj_entity);

      if (obstacles) {
        CollisionRequestDataMultipleObstacles reqData(
            *obstacles, this->parent_map, max_obstacles);
        reqData.setUngroupShapeGroups(ungroup_shape_groups);
        reqData.setUngroupTvObstacles(ungroup_TV_obstacles);
        res = collide_fcl(fclgr, col_time, reqData);
      } else {
        res = collide_fcl(fclgr, col_time);
      }
      if (collision_time) {
        *collision_time = col_time;
      }
      return res;
      break;
    case FCL_COLLISION_ENTITY_TYPE_TVOBJECT:
      tvobj = static_cast<const TimeVariantCollisionObject *>(co.get());
      if (obstacles) {
        CollisionRequestDataMultipleObstacles reqData(
            *obstacles, this->parent_map, max_obstacles);
        reqData.setUngroupShapeGroups(ungroup_shape_groups);
        reqData.setUngroupTvObstacles(ungroup_TV_obstacles);
        res = collide_fcl(tvobj, col_time, reqData);
      } else {
        res = collide_fcl(tvobj, col_time);
      }
      if (collision_time) {
        *collision_time = col_time;
      }
      return res;
      break;
    default:
      throw;
      return true;
      break;
  }
}

// Returns a CollisionChecker of certain kind with all static objects within the
// window and all time-variant obstacles. Ungroups all shape groups. The
// function is not thread-safe

void FCLCollisionChecker::windowQuery_helper(const RectangleAABB &aabb,
                                             ICollisionChecker &ret) const {
  static std::vector<const CollisionObject *> windowQueryRes;

  const FCLCollisionObject *fclco = get_fcl_object_ptr(&aabb);
  if (!fclco) {
    throw;
    return;
  }

  // add all static obstacles within the window

  fcl::CollisionObject<FCL_PRECISION> *fcl_obj =
      fclco->getCollisionObject_fcl().get();

  if (!windowQueryDone) {
    windowQueryRes.reserve(m_cc.numberOfObstacles());
  }
  windowQueryRes.clear();

  CollisionRequestDataWindowQuery reqData(fcl_obj, windowQueryRes,
                                          this->parent_map);

  setUpStaticObstaclesManager();

  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false, FCL_SOLVER_TYPE);  // Disable finding of contact points
  collisionRequest.enable_cost = false;

  CollisionDataWindowQuery<FCL_PRECISION> self_data_static_obstacles(reqData);
  self_data_static_obstacles.request = collisionRequest;

  static_obstacles_.collide(fcl_obj, &self_data_static_obstacles,
                            defaultCollisionFunctionWindowQuery<FCL_PRECISION>);

  for (auto &x : windowQueryRes) {
    ret.addCollisionObject(x->shared_from_this());
  }
  // add all TV obstacles

  for (auto &c : tv_objects_) {
    ret.addCollisionObject(c);
  }
  windowQueryDone = true;
}

bool FCLCollisionChecker::collide_fcl(
    const FCLCollisionObject *col_obj_ptr, int &time_of_collision,
    CollisionRequestDataMultipleObstacles &reqData) const {
  if (!col_obj_ptr) {
    throw;
    return true;
  }
  fcl::CollisionObject<FCL_PRECISION> *col_obj =
      col_obj_ptr->getCollisionObject_fcl().get();
  return this->collide_fcl_helper_simulate_static_subject(
      col_obj, time_of_collision,
      CollisionRequestType::collisionRequestListOfObstacles, &reqData);
}

bool FCLCollisionChecker::collide_fcl(
    const FCLCollisionObjectGroup *col_obj_group, int &time_of_collision,
    CollisionRequestDataMultipleObstacles &reqData) const {
  if (!col_obj_group) {
    throw;
    return true;
  }
  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr;
  col_obj_group->getManager_fcl(mngr);
  return this->collide_fcl_helper_simulate_static_subject(
      mngr, time_of_collision,
      CollisionRequestType::collisionRequestListOfObstacles, &reqData);
}

bool FCLCollisionChecker::collide_fcl(const FCLCollisionObject *col_obj_ptr,
                                      int &time_of_collision) const {
  if (!col_obj_ptr) {
    throw;
    return true;
  }
  fcl::CollisionObject<FCL_PRECISION> *col_obj =
      col_obj_ptr->getCollisionObject_fcl().get();
  return this->collide_fcl_helper_simulate_static_subject(
      col_obj, time_of_collision, CollisionRequestType::collisionRequestBoolean,
      0);
}

bool FCLCollisionChecker::collide_fcl(
    const FCLCollisionObjectGroup *col_obj_group,
    int &time_of_collision) const {
  if (!col_obj_group) {
    throw;
    return true;
  }
  fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr;
  col_obj_group->getManager_fcl(mngr);
  return this->collide_fcl_helper_simulate_static_subject(
      mngr, time_of_collision, CollisionRequestType::collisionRequestBoolean,
      0);
}

bool FCLCollisionChecker::collide_fcl(const TimeVariantCollisionObject *tvobj,
                                      int &time_of_collision) const {
  if (!tvobj) {
    throw;
    return true;
  }

  return collide_fcl_helper_sim_TV_subject(
      *tvobj, time_of_collision, CollisionRequestType::collisionRequestBoolean,
      0);
}

bool FCLCollisionChecker::collide_fcl(
    const TimeVariantCollisionObject *tvobj, int &time_of_collision,
    CollisionRequestDataMultipleObstacles &reqData) const {
  if (!tvobj) {
    throw;
    return true;
  }
  return collide_fcl_helper_sim_TV_subject(
      *tvobj, time_of_collision,
      CollisionRequestType::collisionRequestListOfObstacles, &reqData);
}

// simulate collision with a TV obstacle
bool FCLCollisionChecker::collide_fcl_helper_sim_TV_subject(
    const TimeVariantCollisionObject &tvobj, int &time_of_collision,
    CollisionRequestType reqType, CollisionRequestData *reqData) const {
  bool result = false;
  int sim_start_idx = 0;
  int sim_end_idx = 0;
  //
  if (simulation_dynamic_) {
    sim_start_idx =
        std::min(simulation_time_start_idx_, tvobj.time_start_idx());
    sim_end_idx = std::max(simulation_time_end_idx_, tvobj.time_end_idx());
  } else {
    sim_start_idx = tvobj.time_start_idx();
    sim_end_idx = tvobj.time_end_idx();
  }
  bool cur_res = false;
  int cur_collision_time = -1;

  for (int sim_cur_idx = sim_start_idx; sim_cur_idx <= sim_end_idx;
       sim_cur_idx++) {
    CollisionObjectConstPtr col_obj_ptr = tvobj.getObstacleAtTime(sim_cur_idx);
    if (col_obj_ptr) {
      SolverEntity_FCL *obj_entity;
      FCL_COLLISION_ENTITY_TYPE colent_type =
          get_object_fcl_entity_type(col_obj_ptr, obj_entity);
      if (colent_type == COLLISION_ENTITY_TYPE_FCL_OBJECT) {
        const FCLCollisionObject *fclco =
            static_cast<const FCLCollisionObject *>(obj_entity);
        fcl::CollisionObject<FCL_PRECISION> *col_obj = 0;

        col_obj = (fclco)->getCollisionObject_fcl().get();
        if (col_obj) {
          if (collide_fcl_helper_static_obstaclesEx(
                  col_obj, result, reqType,
                  reqData)) {  // the child function signalled early stopping
            if (result) {
              time_of_collision = sim_cur_idx;
              return result;
            }
          }
          if (result) {  // if it is a first collision, remember time of
                         // collision
            if (cur_collision_time == -1) {
              cur_collision_time = sim_cur_idx;
            }
            cur_res = true;
          }

          if (collide_fcl_helper_dynamic_obstaclesEx(
                  col_obj, sim_cur_idx, result, reqType,
                  reqData)) {  // the child function signalled early stopping
            if (result) {
              time_of_collision = sim_cur_idx;
              return result;
            }
          }
          if (result) {
            if (cur_collision_time == -1) {
              cur_collision_time = sim_cur_idx;
            }
            cur_res = true;
          }
        }

      } else if (colent_type == COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP) {
        const FCLCollisionObjectGroup *fclco_gr =
            static_cast<const FCLCollisionObjectGroup *>(obj_entity);

        fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr = 0;

        fclco_gr->getManager_fcl(mngr);
        if (mngr) {
          if (collide_fcl_helper_static_obstaclesEx(mngr, result, reqType,
                                                    reqData)) {
            if (result) {
              time_of_collision = sim_cur_idx;
              return result;
            }
          }
          if (result) {
            if (cur_collision_time == -1) {
              cur_collision_time = sim_cur_idx;
            }
            cur_res = true;
          }
          if (collide_fcl_helper_dynamic_obstaclesEx(mngr, sim_cur_idx, result,
                                                     reqType, reqData)) {
            if (result) {
              time_of_collision = sim_cur_idx;
              return result;
            }
          }
          if (result) {
            if (cur_collision_time == -1) {
              cur_collision_time = sim_cur_idx;
            }
            cur_res = true;
          }
        }
      } else {
        throw;
        return true;
      }
    }
  }
  time_of_collision = cur_collision_time;
  return cur_res;
}

template <class T>
bool FCLCollisionChecker::collide_fcl_helper_simulate_static_subject(
    T col_entity, int &time_of_collision, CollisionRequestType reqType,
    CollisionRequestData *reqData) const {
  bool cur_result = false;
  bool result = false;
  int cur_time_of_coll = -1;
  if (collide_fcl_helper_static_obstaclesEx(col_entity, result, reqType,
                                            reqData)) {
    if (result) {
      time_of_collision = 0;
      return result;
    }
  }
  if (result) {
    if (cur_time_of_coll == -1) cur_time_of_coll = 0;
    cur_result = true;
  }

  if (simulation_dynamic_) {
    for (int cur_time_idx = simulation_time_start_idx_;
         cur_time_idx <= simulation_time_end_idx_; cur_time_idx++) {
      if ((collide_fcl_helper_dynamic_obstaclesEx(col_entity, cur_time_idx,
                                                  result, reqType, reqData))) {
        if (result) {
          time_of_collision = cur_time_idx;
          return result;
        }
      }
      if (result) {
        if (cur_time_of_coll == -1) cur_time_of_coll = 0;
        cur_result = true;
      }
    }
  }
  time_of_collision = cur_time_of_coll;

  return cur_result;
}

template <class T>
bool FCLCollisionChecker::collide_fcl_helper_dynamic_obstaclesEx(
    T col_entity, int sim_cur_time_idx, bool &result,
    CollisionRequestType reqType, CollisionRequestData *reqData) const {
  // TODO: test parent map feature

  if (reqType == CollisionRequestType::collisionRequestListOfObstacles) {
    setUpParentMap();
    result = false;
    if (!reqData) {
      throw;
      return true;
    }
    if (!(static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
              ->getMaxObstacles())) {
      return true;
    }
    if (!set_up_dynamic_obstacles(sim_cur_time_idx)) {
      static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
          ->setSubjectEntitySize(col_entity);
      static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
          ->setTargetMngrSize(dynamic_obstacles_.size());
      static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
          ->setObstacleCallbackOrder();
      CollisionDataListOfObstacles<FCL_PRECISION> self_data_dyn_obstacles(
          *(static_cast<CollisionRequestDataMultipleObstacles *>(reqData)));
      fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
          1, false, FCL_SOLVER_TYPE);  // Disable finding of contact points
      collisionRequest.enable_cost = false;
      self_data_dyn_obstacles.request = collisionRequest;
      dynamic_obstacles_.collide(
          col_entity, &self_data_dyn_obstacles,
          defaultCollisionFunctionListOfObstacles<FCL_PRECISION>);
      result = self_data_dyn_obstacles.get_result();
      return self_data_dyn_obstacles.self_reqData.getAllObstAdded();
    }
    return false;

  } else if (reqType == CollisionRequestType::collisionRequestBoolean) {
    result = collide_fcl_helper_dynamic_obstacles(col_entity, sim_cur_time_idx);
    return true;
  } else {
    throw;
    return true;
  }
}

template <class T>
bool FCLCollisionChecker::collide_fcl_helper_dynamic_obstacles(
    T col_entity, int sim_cur_time_idx) const {
  bool result = false;
  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false, FCL_SOLVER_TYPE);  // Disable finding of contact points
  collisionRequest.enable_cost = false;
  if (!set_up_dynamic_obstacles(sim_cur_time_idx)) {
    CollisionData<FCL_PRECISION> self_data_dyn_obstacles;
    self_data_dyn_obstacles.request = collisionRequest;
    dynamic_obstacles_.collide(col_entity, &self_data_dyn_obstacles,
                               defaultCollisionFunction<FCL_PRECISION>);
    result = self_data_dyn_obstacles.result.isCollision();
  }
  return result;
}

template <class T>
bool FCLCollisionChecker::collide_fcl_helper_static_obstaclesEx(
    T col_entity, bool &result, CollisionRequestType reqType,
    CollisionRequestData *reqData) const {
  if (reqType == CollisionRequestType::collisionRequestListOfObstacles) {
    setUpParentMap();
    result = false;
    if (!reqData) {
      throw;
      return true;
    }
    if (!(static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
              ->getMaxObstacles())) {
      return true;
    }
    static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
        ->setSubjectEntitySize(col_entity);
    static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
        ->setTargetMngrSize(static_obstacles_.size());
    static_cast<CollisionRequestDataMultipleObstacles *>(reqData)
        ->setObstacleCallbackOrder();
    CollisionDataListOfObstacles<FCL_PRECISION> self_data_static_obstacles(
        *(static_cast<CollisionRequestDataMultipleObstacles *>(reqData)));
    fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
        1, false, FCL_SOLVER_TYPE);  // Disable finding of contact points
    collisionRequest.enable_cost = false;
    self_data_static_obstacles.self_reqData;
    self_data_static_obstacles.request = collisionRequest;
    static_obstacles_.collide(
        col_entity, &self_data_static_obstacles,
        defaultCollisionFunctionListOfObstacles<FCL_PRECISION>);
    result = self_data_static_obstacles.get_result();

    return self_data_static_obstacles.self_reqData.getAllObstAdded();

  } else if (reqType == CollisionRequestType::collisionRequestBoolean) {
    result = collide_fcl_helper_static_obstacles(col_entity);
    return true;
  } else {
    throw;
    return true;
  }
}

template <class T>
bool FCLCollisionChecker::collide_fcl_helper_static_obstacles(
    T col_entity) const {
  bool result = false;
  CollisionData<FCL_PRECISION> self_data_static_obstacles;
  fcl::CollisionRequest<FCL_PRECISION> collisionRequest(
      1, false, FCL_SOLVER_TYPE);  // Disable finding of contact points
  collisionRequest.enable_cost = false;
  self_data_static_obstacles.request = collisionRequest;
  static_obstacles_.collide(col_entity, &self_data_static_obstacles,
                            defaultCollisionFunction<FCL_PRECISION>);
  result = self_data_static_obstacles.result.isCollision();
  return result;
}

template bool FCLCollisionChecker::collide_fcl_helper_simulate_static_subject(
    fcl::CollisionObject<FCL_PRECISION> *col_entity, int &time_of_collision,
    CollisionRequestType reqType, CollisionRequestData *reqData) const;
template bool FCLCollisionChecker::collide_fcl_helper_simulate_static_subject(
    fcl::BroadPhaseCollisionManager<FCL_PRECISION> *col_entity,
    int &time_of_collision, CollisionRequestType reqType,
    CollisionRequestData *reqData) const;

}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
