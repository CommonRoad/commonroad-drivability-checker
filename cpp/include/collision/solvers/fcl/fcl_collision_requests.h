#ifndef COLLISION_FCL_FCL_COLLISION_REQUESTS_H_
#define COLLISION_FCL_FCL_COLLISION_REQUESTS_H_

#include "collision/application.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include "collision/solvers/fcl/fcl_collision_object.h"
#include "collision/solvers/fcl/fcl_collision_object_group.h"

#include <set>

namespace collision {
class CollisionChecker;
typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;
typedef std::shared_ptr<const CollisionChecker> CollisionCheckerConstPtr;

namespace solvers {
namespace solverFCL {
CollisionObject *getParentPointerFromFclObj(
    fcl::CollisionObject<FCL_PRECISION> *fcl_obj);

class CollisionRequestData {};

class CollisionRequestDataMultipleObstacles : public CollisionRequestData {
 public:
  CollisionRequestDataMultipleObstacles(
      std::vector<CollisionObjectConstPtr> &ret_obstacles,
      const std::unordered_map<const CollisionObject *,
                               std::list<CollisionObjectConstPtr>> &parent_map,
      int max_obstacles)
      : m_obstacles(ret_obstacles),
        m_parent_map(parent_map),
        m_max_obstacles(max_obstacles){

        };

  bool addRequestResultObstacle(const CollisionObject *obst);

  bool testPair(std::pair<fcl::CollisionObject<FCL_PRECISION> *,
                          fcl::CollisionObject<FCL_PRECISION> *>
                    pair);

  void setMaxObstacles(int max_obstacles) { m_max_obstacles = max_obstacles; }

  int getMaxObstacles() const { return m_max_obstacles; }

  void setSubjectEntitySize(fcl::CollisionObject<FCL_PRECISION> *obj) {
    m_subject_group_size = -1;  // only for comparison to determine object order
                                // in broadphase callback
  }
  void setSubjectEntitySize(
      fcl::BroadPhaseCollisionManager<FCL_PRECISION> *mngr) {
    m_subject_group_size = mngr->size();
  }

  void setTargetMngrSize(int size) { m_target_mngr_size = size; }

  void setObstacleCallbackOrder(void) {
    m_obstacle_callback_first = m_target_mngr_size < m_subject_group_size;
  }

  bool getObstacleCallbackOrder(void) const {
    return !m_obstacle_callback_first;
  }

  void setUngroupShapeGroups(bool val) { m_ungroup_sg = val; }
  void setUngroupTvObstacles(bool val) { m_ungroup_tvobst = val; }

  void setAllObst_added(void) { m_finished = true; }

  bool getAllObstAdded(void) const { return m_finished; }

 private:
  bool addRequestResultObstacleObject(CollisionObjectConstPtr obj);
  std::vector<CollisionObjectConstPtr> &m_obstacles;
  int m_subject_group_size;
  int m_target_mngr_size;
  const std::unordered_map<const CollisionObject *,
                           std::list<CollisionObjectConstPtr>> &m_parent_map;
  std::unordered_map<const CollisionObject *, bool> m_added_map;
  std::unordered_map<const CollisionObject *, bool> m_added_map_parent;
  std::set<std::pair<fcl::CollisionObject<FCL_PRECISION> *,
                     fcl::CollisionObject<FCL_PRECISION> *>>
      checkedPairs;
  bool m_obstacle_callback_first = false;
  bool m_ungroup_tvobst = false;
  bool m_ungroup_sg = false;
  bool m_finished = false;
  int m_max_obstacles;
};

class CollisionRequestDataWindowQuery : public CollisionRequestData {
 public:
  CollisionRequestDataWindowQuery(
      fcl::CollisionObject<FCL_PRECISION> *subject_object,
      std::vector<const CollisionObject *> &result,
      const std::unordered_map<const collision::CollisionObject *,
                               std::list<CollisionObjectConstPtr>> &parent_map)
      : m_subject_object(subject_object),
        m_result(result),
        m_parent_map(parent_map){};

  void addRequestResultObstacle(const CollisionObject *obst);

  fcl::CollisionObject<FCL_PRECISION> *getSubjectObject(void) const {
    return m_subject_object;
  }

 private:
  fcl::CollisionObject<FCL_PRECISION> *m_subject_object;
  std::vector<const CollisionObject *> &m_result;
  const std::unordered_map<const CollisionObject *,
                           std::list<CollisionObjectConstPtr>> &m_parent_map;
  std::unordered_map<const CollisionObject *, bool> m_added_map;
};

class CollisionRequestDataOverlap : public CollisionRequestData {
 public:
  CollisionRequestDataOverlap(const ICollisionContainer *cont_1,
                              const ICollisionContainer *cont_2)
      : m_cont_1(cont_1), m_cont_2(cont_2){};

  void addRequestResultPair(int o1, int o2);

  void addRequestResultList(std::list<int> list1, std::list<int> list2);

  std::set<std::pair<int, int>> getRequestResultPairs() const;

  int set_debug_pairs(std::vector<std::pair<int, int>> pairs_in) {
    m_Debug_Pairs = pairs_in;
    return 0;
  }

  int get_debug_pairs(std::vector<std::pair<int, int>> &pairs_out) const {
    pairs_out = m_Debug_Pairs;
    return 0;
  }

  const ICollisionContainer *get_cont_1(void) const { return m_cont_1; }
  const ICollisionContainer *get_cont_2(void) const { return m_cont_2; }

  std::set<std::pair<fcl::CollisionObject<FCL_PRECISION> *,
                     fcl::CollisionObject<FCL_PRECISION> *>>
      checkedPairs;

 private:
  const ICollisionContainer *m_cont_1;
  const ICollisionContainer *m_cont_2;

  std::vector<std::pair<int, int>> m_Debug_Pairs;

  std::set<std::pair<int, int>> m_ResultPairs;
};

/// @brief Collision data stores the collision request and the result given by
/// collision algorithm.
// CollisionData struct MUST not be changed
template <typename S>
struct CollisionData {
  CollisionData() { done = false; }

  /// @brief Collision request
  fcl::CollisionRequest<S> request;

  /// @brief Collision result
  fcl::CollisionResult<S> result;

  /// @brief Whether the collision iteration can stop
  bool done;
};

template <typename S>
struct CollisionDataEx : public CollisionData<S> {
  CollisionDataEx() { m_bool_result = false; }
  void set_result(bool res) { m_bool_result = res; }
  bool get_result(void) { return m_bool_result; }

 private:
  bool m_bool_result;
};

template <typename S>
struct CollisionDataOverlap : public CollisionDataEx<S> {
  CollisionRequestDataOverlap &self_reqData;

  CollisionDataOverlap(CollisionRequestDataOverlap &reqData)
      : self_reqData(reqData) {}
};

template <typename S>
struct CollisionDataListOfObstacles : public CollisionDataEx<S> {
  CollisionRequestDataMultipleObstacles &self_reqData;

  CollisionDataListOfObstacles(CollisionRequestDataMultipleObstacles &reqData)
      : self_reqData(reqData) {}
};
template <typename S>
struct CollisionDataWindowQuery : public CollisionDataEx<S> {
  CollisionRequestDataWindowQuery &self_reqData;

  CollisionDataWindowQuery(CollisionRequestDataWindowQuery &reqData)
      : self_reqData(reqData) {}
};

template <typename S>
bool defaultCollisionFunctionWindowQuery(fcl::CollisionObject<S> *o1,
                                         fcl::CollisionObject<S> *o2,
                                         void *cdata_) {
  auto *cdata = static_cast<CollisionDataWindowQuery<S> *>(cdata_);
  const auto &request = cdata->request;

  if (cdata->done) return true;

  const fcl::CollisionObject<S> *subj_obj =
      cdata->self_reqData.getSubjectObject();

  if (o1 == subj_obj)

  {
    cdata->self_reqData.addRequestResultObstacle(
        getParentPointerFromFclObj(o2));
  } else if (o2 == subj_obj) {
    cdata->self_reqData.addRequestResultObstacle(
        getParentPointerFromFclObj(o1));
  }

  return cdata->done;
}
/*
template <typename S>
bool defaultprimitveCollisionFunction(solverFCL::CollisionObject<S>* o1,
solverFCL::CollisionObject<S>* o2, void* cdata_)
{
auto* cdata = static_cast<CollisionData<S>*>(cdata_);
const auto& request = cdata->request;
auto& result = cdata->result;
const CollisionObject* obj1_entity=getParentPointerFromFclObj(o1);
const CollisionObject* obj2_entity=getParentPointerFromFclObj(o2);
if(obj1_entity->collide(*obj2_entity,CollisionRequest(COL_PRIMITIVE)))
{
solverFCL::Contact<double> cont;
result.addContact(cont);
return true;
}
else
return false;
//cdata->result.addContact();

}
*/

template <typename S>
bool defaultCollisionFunction(fcl::CollisionObject<S> *o1,
                              fcl::CollisionObject<S> *o2, void *cdata_) {
  auto *cdata = static_cast<CollisionData<S> *>(cdata_);
  const auto &request = cdata->request;
  auto &result = cdata->result;

  if (cdata->done) return true;

  collide(o1, o2, request, result);

  if (!request.enable_cost && (result.isCollision()) &&
      (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

template <typename S>
bool defaultCollisionFunctionOverlap(fcl::CollisionObject<S> *o1,
                                     fcl::CollisionObject<S> *o2,
                                     void *cdata_) {
  CollisionDataOverlap<S> *cdata =
      static_cast<CollisionDataOverlap<S> *>(cdata_);
  fcl::CollisionRequest<S> *request = &(cdata->request);
  fcl::CollisionResult<S> *result = &(cdata->result);

  if (cdata->done) return true;

  const auto &pair_obj =
      std::pair<fcl::CollisionObject<S> *, fcl::CollisionObject<S> *>(o1, o2);

  // TODO: if collision detected earlier - don't check again!
  auto &checked_pairs = cdata->self_reqData.checkedPairs;
  auto search = checked_pairs.find(pair_obj);
  if (search != checked_pairs.end()) return false;

  fcl::collide(o1, o2, *request, *result);

  checked_pairs.emplace(pair_obj);

  if (cdata->result.isCollision()) {
    (cdata->result).clear();
    const CollisionObject *obj1_entity = getParentPointerFromFclObj(o1);
    const CollisionObject *obj2_entity = getParentPointerFromFclObj(o2);

    if (obj1_entity && obj2_entity) {
      std::list<int> cont1_obj1_list;
      std::list<int> cont1_obj2_list;
      std::list<int> cont2_obj1_list;
      std::list<int> cont2_obj2_list;

      int res_cont1_obj1 =
          cdata->self_reqData.get_cont_1()->queryContainedObjectIndexList(
              obj1_entity, cont1_obj1_list);
      int res_cont1_obj2 =
          cdata->self_reqData.get_cont_1()->queryContainedObjectIndexList(
              obj2_entity, cont1_obj2_list);

      int res_cont2_obj1 =
          cdata->self_reqData.get_cont_2()->queryContainedObjectIndexList(
              obj1_entity, cont2_obj1_list);
      int res_cont2_obj2 =
          cdata->self_reqData.get_cont_2()->queryContainedObjectIndexList(
              obj2_entity, cont2_obj2_list);

      if (!res_cont1_obj1 && !res_cont2_obj1)  // obj1 is in both containers
      {
        cdata->self_reqData.addRequestResultList(cont1_obj1_list,
                                                 cont2_obj1_list);
      }

      if (!res_cont1_obj2 && !res_cont2_obj2)  // obj2 is in both containers
      {
        cdata->self_reqData.addRequestResultList(cont1_obj2_list,
                                                 cont2_obj2_list);
      }
      if (!res_cont1_obj1 &&
          !res_cont2_obj2)  // obj1 in container 1, obj2 in container 2
      {
        cdata->self_reqData.addRequestResultList(cont1_obj1_list,
                                                 cont2_obj2_list);
      }

      if (!res_cont1_obj2 &&
          !res_cont2_obj1)  // obj1 in container 1, obj2 in container 2
      {
        cdata->self_reqData.addRequestResultList(cont1_obj2_list,
                                                 cont2_obj1_list);
      }
    }
  }

  return cdata->done;
}

template <typename S>
bool defaultCollisionFunctionListOfObstacles(fcl::CollisionObject<S> *o1,
                                             fcl::CollisionObject<S> *o2,
                                             void *cdata_) {
  auto *cdata = static_cast<CollisionDataListOfObstacles<S> *>(cdata_);
  const auto &request = cdata->request;

  if (cdata->done) return true;

  fcl::CollisionObject<S> *obst = 0;

  fcl::CollisionObject<S> *subj = 0;

  // TODO: if collision detected earlier - don't check again!
  if (cdata->self_reqData.getObstacleCallbackOrder()) {
    obst = o1;
    subj = o2;
  } else {
    subj = o1;
    obst = o2;
  }

  const auto &pair_obj =
      std::pair<fcl::CollisionObject<S> *, fcl::CollisionObject<S> *>(obst,
                                                                      subj);

  if (!(cdata->self_reqData.testPair(pair_obj))) return false;

  collide(obst, subj, request, cdata->result);

  if (cdata->result.isCollision()) {
    cdata->set_result(true);

    cdata->result.clear();

    const CollisionObject *obst_entity = getParentPointerFromFclObj(obst);
    const CollisionObject *subj_entity = getParentPointerFromFclObj(subj);

    // try to add both, as the order in the callback is undefined
    if (cdata->self_reqData.addRequestResultObstacle(obst_entity)) {
      cdata->self_reqData.setAllObst_added();
      return true;
    }
    if (cdata->self_reqData.addRequestResultObstacle(subj_entity)) {
      cdata->self_reqData.setAllObst_added();
      return true;
    }
  }

  return cdata->done;
}

}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision

#endif /* COLLISION_FCL_FCL_COLLISION_REQUESTS_H_ */
