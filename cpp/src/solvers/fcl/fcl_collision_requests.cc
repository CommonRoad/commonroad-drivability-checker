#include "collision/solvers/fcl/fcl_collision_requests.h"
#include <utility>
#include "collision/collision_checker.h"
#include "collision/solvers/fcl/fcl_helpers.h"

namespace collision {
namespace solvers {
namespace solverFCL {

void CollisionRequestDataOverlap::addRequestResultPair(int o1, int o2) {
  m_ResultPairs.emplace(o1, o2);
}

void CollisionRequestDataOverlap::addRequestResultList(std::list<int> list1,
                                                       std::list<int> list2) {
  for (int el1 : list1) {
    for (int el2 : list2) {
      addRequestResultPair(el1, el2);
    }
  }
}

std::set<std::pair<int, int>>
CollisionRequestDataOverlap::getRequestResultPairs() const {
  return m_ResultPairs;
}

bool CollisionRequestDataMultipleObstacles::testPair(
    std::pair<fcl::CollisionObject<FCL_PRECISION> *,
              fcl::CollisionObject<FCL_PRECISION> *>
        pair) {
  auto &checked_pairs = checkedPairs;
  auto search = checked_pairs.find(pair);
  if (search != checked_pairs.end()) return false;

  checked_pairs.emplace(pair);

  return true;
}

bool CollisionRequestDataMultipleObstacles::addRequestResultObstacleObject(
    CollisionObjectConstPtr obj) {
  // todo: add a dedicated counter to optimize perfomance

  if (m_max_obstacles < 0) {
    m_obstacles.push_back(obj);
    return false;
  } else {
    if ((int)m_obstacles.size() < m_max_obstacles) {
      m_obstacles.push_back(obj);
    }
    return (m_obstacles.size() == m_max_obstacles);
  }
}

bool CollisionRequestDataMultipleObstacles::addRequestResultObstacle(
    const CollisionObject *obst) {
  // todo: subject obstacles to a map, check which is a subject and add the
  // parent todo: routine to test pair for belonging to a group which was
  // already added todo: different options of what to return: with ShapeGroups
  // or raw objects - as a setting

  // don't add the obstacle if already added
  auto already_added = m_added_map.find(obst);
  if (already_added == m_added_map.end()) {
    m_added_map.emplace(obst, true);
    auto it = m_parent_map.find(obst);

    if (it != m_parent_map.end()) {
      if (m_ungroup_sg && m_ungroup_tvobst) {
        if (addRequestResultObstacleObject(obst->shared_from_this()))
          return true;
      } else {
        bool found_sg = false;
        bool found_tvobst = false;

        // find all entries corresponding to the obstacle (obstacle itself and
        // its parents in the CC)

        for (auto &member_obstacle : it->second) {
          const CollisionObject *memb_obst = member_obstacle.get();
          auto already_added_parent = m_added_map_parent.find(memb_obst);
          if (already_added_parent == m_added_map_parent.end()) {
            m_added_map_parent.emplace(memb_obst, true);

            FCL_COLLISION_ENTITY_TYPE memb_obst_entity_type =
                get_object_fcl_entity_type(memb_obst);

            switch (memb_obst_entity_type) {
              case FCL_COLLISION_ENTITY_TYPE_TVOBJECT:
                if (!m_ungroup_tvobst)  // if registered tv obstacle parents
                                        // should be added as a group
                {
                  if (addRequestResultObstacleObject(member_obstacle))
                    return true;
                } else  // or as an individual object
                {
                  found_tvobst = true;
                }
                break;
              case COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP:
                if (!m_ungroup_sg) {
                  if (addRequestResultObstacleObject(member_obstacle))
                    return true;
                } else {
                  found_sg = true;
                }
                break;
              default:
                if (addRequestResultObstacleObject(member_obstacle))
                  return true;
            }
          }
        }
        if (found_sg || found_tvobst) {
          if (addRequestResultObstacleObject(obst->shared_from_this()))
            return true;
        }
      }

    } else {
      // std::cout << "parent obj not found";
#if ENABLE_COLLISION_TESTS
//			collision::test::log_parentmap_failure(obst->shared_from_this());
#endif
    }
  }
  return false;
}

void CollisionRequestDataWindowQuery::addRequestResultObstacle(
    const CollisionObject *obst) {
  auto already_added = m_added_map.find(obst);
  if (already_added == m_added_map.end()) {
    m_added_map.emplace(obst, true);
    // auto it = m_parent_map.find(obst);
    m_result.push_back(obst);
    // m_cc->addCollisionObject(obst->shared_from_this());
    // if( it != m_parent_map.end() )
    //{

    //	for(auto &member_obstacle: it->second)
    //	{
    //		m_cc->addCollisionObject(member_obstacle);
    //	}
    //}
  }
}

}  // namespace solverFCL
}  // namespace solvers
}  // namespace collision
