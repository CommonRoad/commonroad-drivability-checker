#ifndef CPP_COLLISION_INCLUDE_COLLISION_GRID_FCL_H_
#define CPP_COLLISION_INCLUDE_COLLISION_GRID_FCL_H_

#include "collision/collision_object.h"

#include <fcl/broadphase/broadphase_SSaP.h>
#include <fcl/broadphase/broadphase_SaP.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_interval_tree.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <stdint.h>
#include <boost/align/aligned_allocator.hpp>
#include "collision/shape_group.h"
#include "collision/solvers/primitive_collision_queries.h"

#include <fcl/narrowphase/collision_object.h>

#include "collision/narrowphase/detail/aabb.h"
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"
#include "collision/solvers/sat2d/aabb_sat2d.h"
#include "collision/solvers/sat2d/obb_sat2d.h"

#include "collision/solvers/accelerators/declarations.h"
#include "collision/solvers/accelerators/detail/common.h"
#include "collision/solvers/accelerators/detail/common_impl.h"

namespace collision {
namespace detail {
namespace accelerators {
class ContainerCollisionRequestDataFCL {
 public:
  ContainerCollisionRequestDataFCL(aligned_vector<int>& candidates)
      : candidates_(&candidates){

        };
  aligned_vector<int>* candidates_;
};

template <typename S>
bool ContainerFunctionWindowQueryFCL(fcl::CollisionObject<S>* o1,
                                     fcl::CollisionObject<S>* o2,
                                     void* cdata_) {
  auto* cdata = static_cast<ContainerCollisionRequestDataFCL*>(cdata_);
  if (o1->getUserData()) {
    cdata->candidates_->push_back(*((int*)o1->getUserData()));
  } else if (o2->getUserData()) {
    cdata->candidates_->push_back(*((int*)o2->getUserData()));
  }
  return false;
};

enum FCLContainerBroadphaseType {
  FCL_BT_DYNAMIC_TREE = 0,
  FCL_BT_SPATIALHASH = 1,
  FCL_BT_INTERVALTREE = 2,
  FCL_BT_SSAP = 3,
  FCL_BT_SAP = 4
};

typedef Eigen::Matrix<uint64_t, Eigen::Dynamic, 1> VectorXi64;

class ContainerFCL {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContainerFCL(aligned_vector<collision::CollisionObject*> objects_in,
               const ContainerSettings& sett) {
    constructHelper();

    for (auto obj : objects_in) {
      addObject(obj);
    }
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (auto obj : objects_in) {
      AABB aabb = getAABB(obj);

      min_x = std::min(min_x, aabb.x_min);
      min_y = std::min(min_y, aabb.y_min);
      max_x = std::max(max_x, aabb.x_max);
      max_y = std::max(max_y, aabb.y_max);
    }

    double gridlen = max_x - min_x;
    double cell_size = gridlen / sett.num_cells;

    switch (sett.fcl_broadphase_type) {
      case FCL_BT_DYNAMIC_TREE:
        broadphase_manager_ = new fcl::DynamicAABBTreeCollisionManager<double>;
        break;
      case FCL_BT_SPATIALHASH:

        broadphase_manager_ = new fcl::SpatialHashingCollisionManager<double>(
            cell_size, fcl::Vector3<double>(min_x, min_y, -FCL_HEIGHT),
            fcl::Vector3<double>(max_x, max_y, FCL_HEIGHT),
            sett.fcl_num_buckets);
        break;
      case FCL_BT_INTERVALTREE:

        broadphase_manager_ = new fcl::IntervalTreeCollisionManager<double>;

        break;
      case FCL_BT_SAP:

        broadphase_manager_ = new fcl::SaPCollisionManager<double>;

        break;
      case FCL_BT_SSAP:

        broadphase_manager_ = new fcl::SSaPCollisionManager<double>;

        break;
      default:
        throw std::invalid_argument("Invalid FCL broadphase manager ID");
    }
    finalizeCreation();
  }

  ContainerFCL(const ShapeGroup* sg_ptr, const ContainerSettings& sett) {
    constructHelper();
    for (auto obj : sg_ptr->unpack()) {
      addObject(obj.get());
    }

    auto sg_aabb = sg_ptr->getAABB();
    double min_x = sg_aabb->min()(0);
    double min_y = sg_aabb->min()(1);
    double max_x = sg_aabb->max()(0);
    double max_y = sg_aabb->max()(1);

    double gridlen = max_x - min_x;
    double cell_size = gridlen / sett.num_cells;

    switch (sett.fcl_broadphase_type) {
      case FCL_BT_DYNAMIC_TREE:
        broadphase_manager_ = new fcl::DynamicAABBTreeCollisionManager<double>;
        break;
      case FCL_BT_SPATIALHASH:

        broadphase_manager_ = new fcl::SpatialHashingCollisionManager<double>(
            cell_size, fcl::Vector3<double>(min_x, min_y, -FCL_HEIGHT),
            fcl::Vector3<double>(max_x, max_y, FCL_HEIGHT),
            sett.fcl_num_buckets);
        break;
      case FCL_BT_INTERVALTREE:

        broadphase_manager_ = new fcl::IntervalTreeCollisionManager<double>;

        break;
      case FCL_BT_SAP:

        broadphase_manager_ = new fcl::SaPCollisionManager<double>;

        break;
      case FCL_BT_SSAP:

        broadphase_manager_ = new fcl::SSaPCollisionManager<double>;

        break;
      default:
        throw std::invalid_argument("Invalid FCL broadphase manager ID");
    }

    finalizeCreation();
  }

  virtual ~ContainerFCL() {
    if (broadphase_manager_) delete broadphase_manager_;
  }

  int addObject(const CollisionObject* obj) {
    if (obj->getCollisionObjectClass() == collision::OBJ_CLASS_SHAPE) {
      AABB aabb = getAABB(obj);

      registerObjectId(obj);
      aabbs_.push_back(aabb);
      ObjectProxy proxy = CreateProxy(obj, aabbs_.back());
      objects_.push_back(proxy);
      return 0;
    }
    return -1;
  }

  int checkCollision(const CollisionObject* obj,
                     const ContainerCollisionRequest& req) {
    bool res = false;
    TIMER_START(TIMER_grid_hashing);
    AABB aabb = getAABB(obj);

    ObjectProxy proxy = CreateProxy(obj, aabb);
    TIMER_STOP(TIMER_grid_hashing);
    aligned_vector<int> collision_candidates;

    {
      STACK_TIMER tim(TIMER_grid_candidates);

      numchecks_ += objects_.size();
      getCandidates(proxy, collision_candidates);
    }
    if (req.enable_verification) {
      aligned_vector<int> collision_candidates2;
      {
        for (int i = 0; i < objects_optimized_id_.size(); i++) {
          // ground truth collisions between AABBs not including border
          // collisions
          if (AABB_SAT2D(getAABB(object_by_id_[objects_optimized_id_[i]]))
                  .collides_eps(AABB_SAT2D(aabb), -1e-11)) {
            collision_candidates2.push_back(objects_optimized_id_[i]);
          }
        }
      }

      std::sort(collision_candidates2.begin(), collision_candidates2.end());
      std::set<int> collision_candidates_set;
      for (auto el : collision_candidates) {
        collision_candidates_set.insert(el);
      }
      std::sort(collision_candidates.begin(), collision_candidates.end());
      int found_count = std::unique(collision_candidates.begin(),
                                    collision_candidates.end()) -
                        collision_candidates.begin();
      int real_count = std::unique(collision_candidates2.begin(),
                                   collision_candidates2.end()) -
                       collision_candidates2.begin();

      volatile int missed = real_count - found_count;

      bool bMissed = false;

      for (auto el : collision_candidates2) {
        if (collision_candidates_set.find(el) ==
            collision_candidates_set.end()) {
          bMissed = true;
          break;
        }
      }

      if (missed > 0 || bMissed) {
        std::cout << "broadphase verification: miss\n";
        throw std::exception();
        return -1;
      }
    }
    {
      STACK_TIMER tim(TIMER_grid_narrowphase);
      collision_checked_id_.fill(0);
      int numcands_local = 0;
      if (obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX && all_obbs_) {
        const RectangleOBB* obb1 = static_cast<const RectangleOBB*>(obj);
        std::vector<OBB_SAT2D, Eigen::aligned_allocator<OBB_SAT2D>>
            obb1_fast_container;
        obb1_fast_container.emplace_back(obb1->center(), obb1->r_x() * 2,
                                         obb1->r_y() * 2, obb1->local_x_axis(),
                                         obb1->local_y_axis());

        for (auto el : collision_candidates) {
          if ((el < 0) || (el >= fast_obbs_.size())) {
            std::cout << "internal error";
            throw std::exception();
          }
          if (!collision_checked_id_(el)) {
            numcands_local += 1;
            collision_checked_id_(el) = 1;
            if (fast_obbs_[el].overlaps(*(obb1_fast_container.begin())))

            {
              numcands_ += numcands_local;
              return 1;
            }
          }
        }
      } else {
        for (auto el : collision_candidates) {
          if (!collision_checked_id_(el)) {
            numcands_local += 1;
            collision_checked_id_(el) = 1;

            if (object_by_id_[el]->collide(*obj)) {
              numcands_ += numcands_local;
              return 1;
            }
          }
        }
      }
      numcands_ += numcands_local;
    }
    return 0;
  }

  int numcands() const { return numcands_; }

  int numchecks() const { return numchecks_; }

 private:
  inline int finalizeCreation(void) {
    int total_objects = 0;
    total_objects += objects_.size();

    objects_optimized_id_.clear();
    object_by_id_.clear();
    object_by_id_.resize(object_ids_.size());
    collision_checked_id_ = Eigen::VectorXi::Zero(object_ids_.size());

    masks_optimized_ = VectorXi64::Zero(total_objects);
    res_mask_optimized_ = Eigen::VectorXi::Zero(total_objects);

    all_obbs_ = true;
    for (auto el : objects_) {
      if (all_obbs_ &&
          el.obj_ptr->getCollisionObjectType() != OBJ_TYPE_OBB_BOX) {
        all_obbs_ = false;
      }

      objects_optimized_id_.push_back(getObjectId(el.obj_ptr));
      object_by_id_[objects_optimized_id_.back()] = el.obj_ptr;
      masks_optimized_[objects_optimized_id_.size() - 1] = el.mask;
      auto curAABB = &aabbs_[objects_optimized_id_.back()];
      collision::RectangleAABBConstPtr rect(new collision::RectangleAABB(
          (curAABB->x_max - curAABB->x_min) / 2,
          (curAABB->y_max - curAABB->y_min) / 2,
          Eigen::Vector2d((curAABB->x_max + curAABB->x_min) / 2,
                          (curAABB->y_max + curAABB->y_min) / 2)));
      fcl_object_geometries_.emplace_back(rect->createFCLCollisionGeometry());
      fcl_objects_.emplace_back(
          fcl_object_geometries_.back(),
          collision::solvers::solverFCL::FCLTransform::fcl_get_3d_translation(
              rect->center()));
    }
    for (auto id : objects_optimized_id_) {
      fcl_objects_[id].setUserData(&objects_optimized_id_[id]);
      broadphase_manager_->registerObject(&fcl_objects_[id]);
    }

    broadphase_manager_->setup();
    return 0;
  }

  inline int getCandidates(ObjectProxy& proxy,
                           aligned_vector<int>& candidates) {
    AABB curAABB = getAABB(proxy.obj_ptr);
    collision::RectangleAABB rect(
        (curAABB.x_max - curAABB.x_min) / 2,
        (curAABB.y_max - curAABB.y_min) / 2,
        Eigen::Vector2d((curAABB.x_max + curAABB.x_min) / 2,
                        (curAABB.y_max + curAABB.y_min) / 2));
    auto cur_geom = std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>>(
        rect.createFCLCollisionGeometry());
    auto cur_obj = fcl::CollisionObject<FCL_PRECISION>(
        cur_geom,
        collision::solvers::solverFCL::FCLTransform::fcl_get_3d_translation(
            rect.center()));
    cur_obj.setUserData(0);
    ContainerCollisionRequestDataFCL reqData(candidates);
    broadphase_manager_->collide(
        &cur_obj, &reqData, ContainerFunctionWindowQueryFCL<FCL_PRECISION>);
    return 0;
  }

  inline int getObjectId(const collision::CollisionObject* obj) {
    auto search = object_ids_.find(obj);
    if (search != object_ids_.end()) {
      return search->second;
    } else
      throw std::exception();
  }

  inline uint64_t get_mask(const AABB& obj) { return 0; }

  inline ObjectProxy CreateProxy(const CollisionObject* obj,
                                 const collision::AABB& aabb) {
    uint64_t mask = get_mask(aabb);
    return ObjectProxy{mask, const_cast<CollisionObject*>(obj)};
    // return ObjectProxy{mask,const_cast<CollisionObject*>(obj), aabb};
  }

  int registerObjectId(const CollisionObject* obj) {
    object_ids_.insert(std::make_pair(obj, (int)object_ids_.size()));
    if (obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
      const collision::RectangleOBB* obb_in =
          static_cast<const collision::RectangleOBB*>(obj);

      fast_obbs_.emplace_back(obb_in->center(), obb_in->r_x() * 2,
                              obb_in->r_y() * 2, obb_in->local_x_axis(),
                              obb_in->local_y_axis());
    }
    return 0;
  }

  inline AABB getAABB(const CollisionObject* obj) {
    if (obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
      AABB a = (static_cast<const RectangleOBB*>(obj))->getAABB_fast();
      return a;
    } else {
      auto aabb_fcl = obj->getAABB();
      return AABB(*aabb_fcl);
    }
  }

  fcl::BroadPhaseCollisionManagerd* broadphase_manager_;

  std::vector<AABB> aabbs_;
  std::vector<ObjectProxy> objects_;

  std::vector<std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>>>
      fcl_object_geometries_;
  std::vector<fcl::CollisionObject<FCL_PRECISION>> fcl_objects_;

  VectorXi64 masks_optimized_;
  Eigen::VectorXi res_mask_optimized_;
  aligned_vector<int> objects_optimized_id_;
  aligned_vector<CollisionObject*> object_by_id_;
  std::vector<OBB_SAT2D, Eigen::aligned_allocator<OBB_SAT2D>> fast_obbs_;
  Eigen::VectorXi collision_checked_id_;

  std::map<const CollisionObject*, int> object_ids_;

  bool all_obbs_;

  int numchecks_;
  int numcands_;

  void constructHelper(void) {
    numchecks_ = 0;
    numcands_ = 0;
    broadphase_manager_ = 0;
  }
};
}  // namespace accelerators
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_GRID_FCL_H_ */
