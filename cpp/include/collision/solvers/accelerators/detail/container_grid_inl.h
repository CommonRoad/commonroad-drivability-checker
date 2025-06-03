#ifndef CPP_COLLISION_INCLUDE_COLLISION_GRID_VIRTUAL_H_
#define CPP_COLLISION_INCLUDE_COLLISION_GRID_VIRTUAL_H_

#include <stdint.h>
#include "collision/collision_object.h"
#include "collision/shape_group.h"

#include "collision/solvers/accelerators/declarations.h"
#include "collision/solvers/accelerators/detail/common.h"
#include "collision/solvers/accelerators/detail/common_impl.h"
#include "collision/solvers/accelerators/detail/container_grid_common.h"
#include "collision/solvers/sat2d/aabb_sat2d.h"
#include "collision/solvers/sat2d/sat2d_checks.h"

namespace collision {
namespace detail {
namespace accelerators {

template <class DIRECTION = HorizontalGrid>
inline AABB getAABB(const CollisionObject* obj) {
  if (obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
    AABB a = (static_cast<const RectangleOBB*>(obj))->getAABB_fast();
    return a;
  } else {
    auto aabb_fcl = obj->getAABB();
    return AABB(*aabb_fcl);
  }
}

template <class DIRECTION = HorizontalGrid>
inline AABB getAABB(const ShapeGroup* sg_ptr) {
  auto sg_aabb = sg_ptr->getAABB();
  return AABB(*sg_aabb);
}

inline int getBestGridOrientationAndBounds(const ShapeGroup* sg_ptr,
                                           AABB& bounds, int& orientation) {
  bounds = getAABB<HorizontalGrid>(sg_ptr);
  if (bounds.x_max - bounds.x_min > bounds.y_max - bounds.y_min) {
    orientation = 0;
  } else {
    orientation = 1;
  }
  return 0;
}

inline int getBestGridOrientationAndBounds(
    aligned_vector<collision::CollisionObject*> objects_in, AABB& bounds,
    int& orientation) {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::min();
  if (objects_in.size() == 0) {
    min_x = 0;
    max_x = 0;
    min_y = 0;
    max_y = 0;
  }

  for (auto obj : objects_in) {
    AABB aabb = getAABB(obj);

    min_x = std::min(min_x, aabb.x_min);
    min_y = std::min(min_y, aabb.y_min);
    max_x = std::max(max_x, aabb.x_max);
    max_y = std::max(max_y, aabb.y_max);
  }

  bounds = AABB(min_x, max_x, min_y, max_y);
  if (bounds.x_max - bounds.x_min > bounds.y_max - bounds.y_min) {
    orientation = 0;
  } else {
    orientation = 1;
    bounds.swapAxes();
  }
  return 0;
}

class GridCell {
 public:
  int addObject(ObjectProxy obj) {
    objects.push_back(obj);
    return 0;
  }

  int getCandidates(ObjectProxy& proxy,
                    std::set<CollisionObject*>& candidates) {
    for (auto obj : objects) {
      if (proxy.mask & obj.mask) {
        candidates.insert(obj.obj_ptr);
      }
    }
    return 0;
  }

  std::vector<ObjectProxy> objects;
};

typedef Eigen::Matrix<uint64_t, Eigen::Dynamic, 1> VectorXi64;

template <class DIRECTION = HorizontalGrid>
class ContainerGrid {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ContainerGrid(AABB* rotated_bounds, const ContainerSettings& sett) {
    constructHelper(rotated_bounds->x_min, rotated_bounds->x_max,
                    rotated_bounds->y_min, rotated_bounds->y_max, sett);
  }

  ContainerGrid(const ShapeGroup* sg_ptr, AABB* rotated_bounds,
                const ContainerSettings& sett) {
    constructHelper(rotated_bounds->x_min, rotated_bounds->x_max,
                    rotated_bounds->y_min, rotated_bounds->y_max, sett);
    for (auto obj : sg_ptr->unpack()) {
      addObject(obj.get());
    }
    finalizeCreation();
  }

  ContainerGrid(aligned_vector<collision::CollisionObject*> objects_in,
                AABB* rotated_bounds, const ContainerSettings& sett) {
    constructHelper(rotated_bounds->x_min, rotated_bounds->x_max,
                    rotated_bounds->y_min, rotated_bounds->y_max, sett);

    for (auto obj : objects_in) {
      addObject(obj);
    }
    finalizeCreation();
  };

  int addObject(const CollisionObject* obj) {
    if (obj->getCollisionObjectClass() == collision::OBJ_CLASS_SHAPE) {
      AABB aabb = getAABB(obj);

      int cell_start = getCell_first(aabb.x_min);
      int cell_end = getCell_first(aabb.x_max);
      registerObjectId(obj);
      aabbs_.push_back(aabb);
      aabbs_2_.emplace_back(aabb);

      ObjectProxy proxy = CreateProxy(obj, aabbs_.back());
      for (int i = cell_start; i <= cell_end; i++) {
        cells_[i].addObject(proxy);
      }
      return 0;
    }
    return -1;
  }

  int numcands() const { return numcands_; }

  int numchecks() const { return numchecks_; }

  int checkCollision(const CollisionObject* obj,
                     const ContainerCollisionRequest& req) {
    bool res = false;
    bool skip_obb_bbox = !req.test_obb_bbox;
    TIMER_START(TIMER_grid_hashing);
    AABB aabb = getAABB(obj);
    AABB_SAT2D aabb2(aabb);

    int cell_start = getCell_first(aabb.x_min);
    int cell_end = getCell_first(aabb.x_max);
    ObjectProxy proxy = CreateProxy(obj, aabb);
    TIMER_STOP(TIMER_grid_hashing);
    aligned_vector<int> collision_candidates;
    aligned_vector<int> collision_candidates2;

    {
      STACK_TIMER tim(TIMER_grid_candidates);

      for (int i = cell_start; i <= cell_end; i++) {
        numchecks_ += cells_[i].objects.size();
        getCandidates(proxy, i, collision_candidates);
      }
    }
    if (req.enable_verification) {
      aligned_vector<int> collision_candidates2;
      {
        for (int i = 0; i < objects_optimized_id_.size(); i++) {
          // ground truth collisions between AABBs excluding border collisions
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
      volatile STACK_TIMER tim(TIMER_grid_narrowphase);
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
            collision_checked_id_(el) = 1;
            numcands_local += 1;

            if (skip_obb_bbox || aabbs_2_[el].collides(aabb2)) {
              if (fast_obbs_[el].overlaps(*(obb1_fast_container.begin()))) {
                numcands_ += numcands_local;
                return 1;
              }
            }
          }
        }
      } else if (optimize_triangles_ &&
                 obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX &&
                 all_triangles_) {
        const RectangleOBB* obb1 = static_cast<const RectangleOBB*>(obj);
        std::vector<OBB_SAT2D, Eigen::aligned_allocator<OBB_SAT2D>>
            obb1_fast_container;
        obb1_fast_container.emplace_back(obb1->center(), obb1->r_x() * 2,
                                         obb1->r_y() * 2, obb1->local_x_axis(),
                                         obb1->local_y_axis());

        for (auto el : collision_candidates) {
          if (!collision_checked_id_(el)) {
            collision_checked_id_(el) = 1;

            numcands_local += 1;

            if (aabbs_2_[el].collides(aabb2)) {
              collision_candidates2.push_back(el);
            }
          }
        }
        for (auto el : collision_candidates2) {
          if (sat2dChecks::overlaps(fast_triangles_[el],
                                    (*(obb1_fast_container.begin())))) {
            numcands_ += numcands_local;
            return 1;
          }
        }
      } else {
        for (auto el : collision_candidates) {
          if (!collision_checked_id_(el)) {
            collision_checked_id_(el) = 1;

            numcands_local += 1;

            if (aabbs_2_[el].collides(aabb2)) {
              collision_candidates2.push_back(el);
            }
          }
        }
        for (auto el : collision_candidates2) {
          if (object_by_id_[el]->collide(*obj)) {
            numcands_ += numcands_local;
            return 1;
          }
        }
      }
      numcands_ += numcands_local;
    }
    return 0;
  }
  int windowQuery(AABB& aabb, aligned_vector<int>& candidates) {
    TIMER_START(TIMER_grid_hashing);
    AABB_SAT2D aabb2(aabb);
    int cell_start = getCell_first(aabb.x_min);
    int cell_end = getCell_first(aabb.x_max);
    ObjectProxy proxy = CreateProxy(0, aabb);
    aligned_vector<int> collision_candidates;
    TIMER_STOP(TIMER_grid_hashing);
    {
      STACK_TIMER tim(TIMER_grid_candidates);

      for (int i = cell_start; i <= cell_end; i++) {
        numchecks_ += cells_[i].objects.size();
        getCandidates(proxy, i, collision_candidates);
      }
    }
    collision_checked_id_.fill(0);
    int numcands_local = 0;
    for (auto el : collision_candidates) {
      if (!collision_checked_id_(el)) {
        collision_checked_id_(el) = 1;

        if (aabbs_2_[el].collides(aabb2)) {
          numcands_local += 1;
          candidates.push_back(el);
        }
      }
    }
    numcands_ += numcands_local;
    return 0;
  }

 private:
  inline int getCandidates(ObjectProxy& proxy, int cell_ind,
                           aligned_vector<int>& candidates) {
    uint64_t proxy_mask = proxy.mask;

    int cell_start_idx = cell_start_[cell_ind];
    int cell_end_idx = cell_end_[cell_ind];
    if (cell_end_idx < cell_start_idx) return 0;
    auto object_iterator = objects_optimized_id_.begin() + cell_start_idx;
    uint64_t* mask_iterator = masks_optimized_.data() + cell_start_idx;

    for (int obj_idx = cell_start_idx; obj_idx <= cell_end_idx; obj_idx++) {
      if (*mask_iterator & proxy_mask) {
        candidates.push_back(*object_iterator);
      }
      object_iterator++;
      mask_iterator++;
    }
    return 0;
  }

  inline int getObjectId(const collision::CollisionObject* obj) {
    auto search = object_ids_.find(obj);
    if (search != object_ids_.end()) {
      return search->second;
    } else
      throw std::exception();
  }

  inline int finalizeCreation(void) {
    cell_start_.resize(cells_.size());
    cell_end_.resize(cells_.size());
    int total_objects = 0;
    for (auto cell : cells_) {
      total_objects += cell.objects.size();
    }
    objects_optimized_id_.clear();
    object_by_id_.clear();
    object_by_id_.resize(object_ids_.size());
    collision_checked_id_ = Eigen::VectorXi::Zero(object_ids_.size());

    masks_optimized_ = VectorXi64::Zero(total_objects);
    res_mask_optimized_ = Eigen::VectorXi::Zero(total_objects);

    all_obbs_ = true;
    all_triangles_ = optimize_triangles_;

    for (int i = 0; i < cells_.size(); i++) {
      cell_start_[i] = objects_optimized_id_.size();
      for (auto el : cells_[i].objects) {
        if (all_obbs_ &&
            el.obj_ptr->getCollisionObjectType() != OBJ_TYPE_OBB_BOX) {
          all_obbs_ = false;
        }

        if (all_triangles_ &&
            el.obj_ptr->getCollisionObjectType() != OBJ_TYPE_TRIANGLE) {
          all_triangles_ = false;
        }

        objects_optimized_id_.push_back(getObjectId(el.obj_ptr));
        object_by_id_[objects_optimized_id_.back()] = el.obj_ptr;
        masks_optimized_[objects_optimized_id_.size() - 1] = el.mask;
      }
      cell_end_[i] = objects_optimized_id_.size() - 1;
    }
    return 0;
  }

  inline uint64_t get_mask(const AABB& obj) {
    int cell_second_start = getCell_second(obj.y_min);
    int cell_second_end = getCell_second(obj.y_max);
    uint64_t mask = 0;
    uint64_t cur_one = (uint64_t)0x1 << cell_second_start;
    for (int i = cell_second_start; i <= cell_second_end; i++) {
      mask |= cur_one;
      cur_one <<= 1;
    }
    return mask;
  }

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
    if (optimize_triangles_) {
      if (obj->getCollisionObjectType() == OBJ_TYPE_TRIANGLE) {
        const collision::Triangle* tri_in =
            static_cast<const collision::Triangle*>(obj);

        fast_triangles_.emplace_back(*tri_in);
      }
    }
    return 0;
  }

  static inline AABB getAABB(const CollisionObject* obj) {
    if (obj->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
      AABB a = (static_cast<const RectangleOBB*>(obj))->getAABB_fast();
      return a;
    } else {
      auto aabb_fcl = obj->getAABB();
      return AABB(*aabb_fcl);
    }
  }

  inline int getCell_first(double first) {
    if (first < first_start_) return 0;
    if (first >= first_end_) return cells_.size() - 1;
    return std::floor((first - first_start_) * inv_cell_size_first_) + 1;
  }

  inline int getCell_second(double second) {
    if (second < second_start_) return 0;
    if (second >= second_end_) return cells_second_size_full - 1;
    return std::floor((second - second_start_) * inv_cell_size_second_) + 1;
  }

  static constexpr int cells_second_size = 62;
  static constexpr int cells_second_size_full = cells_second_size + 2;
  std::vector<GridCell> cells_;
  std::vector<AABB> aabbs_;
  std::vector<AABB_SAT2D> aabbs_2_;

  VectorXi64 masks_optimized_;
  Eigen::VectorXi res_mask_optimized_;
  aligned_vector<int> objects_optimized_id_;
  aligned_vector<CollisionObject*> object_by_id_;
  std::vector<OBB_SAT2D, Eigen::aligned_allocator<OBB_SAT2D>> fast_obbs_;
  Eigen::VectorXi collision_checked_id_;

  std::map<const CollisionObject*, int> object_ids_;

  aligned_vector<int> cell_start_;
  aligned_vector<int> cell_end_;

  std::vector<Triangle_SAT2D, Eigen::aligned_allocator<Triangle_SAT2D>>
      fast_triangles_;
  bool all_obbs_;
  bool all_triangles_;

  bool optimize_triangles_;

  int numcells_;
  double inv_cell_size_first_;
  double inv_cell_size_second_;
  double first_start_;
  double first_end_;
  double gridlen_;
  double second_start_;
  double second_end_;
  double gridlen_second_;

  int numchecks_;
  int numcands_;

  void constructHelper(double x_start, double x_end, double y_start,
                       double y_end, const ContainerSettings& sett) {
    numcells_ = sett.num_cells;
    cells_.resize(numcells_ + 2);
    first_end_ = x_end;
    first_start_ = x_start;
    gridlen_ = first_end_ - first_start_;
    second_end_ = y_end;
    second_start_ = y_start;
    gridlen_second_ = second_end_ - second_start_;
    numchecks_ = 0;
    numcands_ = 0;
    inv_cell_size_first_ = (double)numcells_ / gridlen_;
    inv_cell_size_second_ = (double)cells_second_size / gridlen_second_;
    optimize_triangles_ = sett.optimize_triangles;
  }
};
}  // namespace accelerators
}  // namespace detail
}  // namespace collision

#endif /* CPP_COLLISION_INCLUDE_COLLISION_GRID_VIRTUAL_H_ */
