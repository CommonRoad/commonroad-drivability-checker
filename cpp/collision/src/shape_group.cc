#include "collision/shape_group.h"

#include "collision/solvers/fcl/fcl_collision_queries.h"
#include "collision/solvers/fcl/fcl_helpers.h"

#include <limits>

namespace collision {

ShapeGroupConstPtr ShapeGroup::windowQuery(
    collision::RectangleAABBConstPtr aabb) const {
  ShapeGroupPtr sg_new(new ShapeGroup());
  for (auto el : this->shapes_) {
    if (el->BVCheck(aabb)) {
      sg_new->addToGroup(el);
    }
  }
  return sg_new;
}

std::shared_ptr<const collision::RectangleAABB> ShapeGroup::getAABB() const {
  if (shapes_.size() == 0)
    return RectangleAABBConstPtr(new RectangleAABB(0, 0));
  else {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (auto el : shapes_) {
      auto cur_aabb = el->getAABB();
      double cur_min_x = cur_aabb->min()(0);
      double cur_min_y = cur_aabb->min()(1);
      double cur_max_x = cur_aabb->max()(0);
      double cur_max_y = cur_aabb->max()(1);
      min_x = std::min(min_x, cur_min_x);
      min_y = std::min(min_y, cur_min_y);
      max_x = std::max(max_x, cur_max_x);
      max_y = std::max(max_y, cur_max_y);
    }
    Eigen::Vector2d center((min_x + max_x) / 2, (min_y + max_y) / 2);
    double radius_x = (max_x - min_x) / 2;
    double radius_y = (max_y - min_y) / 2;
    return RectangleAABBConstPtr(new RectangleAABB(radius_x, radius_y, center));
  }
}

int ShapeGroup::getCollisionObjects(
    std::vector<fcl::CollisionObject<FCL_PRECISION> *> &obj_vec) const {
  // obj_vec.clear();
  for (auto i : shapes_) {
    const FCLCollisionObject *i_obj = get_fcl_object_ptr(i.get());
    fcl::CollisionObject<FCL_PRECISION> *obj =
        i_obj->getCollisionObject_fcl().get();  // TODO: thread-safety
    if (obj)
      obj_vec.push_back(obj);
    else
      return 1;
  }
  return 0;
}

/*!
 \brief Returns lists of all objects from the other ShapeGroup that collide with
 each object of this ShapeGroup \param[in] sg - second ShapeGroup for the
 intersection test

 The output vector has as many elements as this ShapeGroup contains.
 Each element of the output vector is a set of indexes of all objects of the
 second ShapeGroup that collide with the object corresponding to the element.

*/

std::vector<std::set<int>> ShapeGroup::overlapMap(const ShapeGroup &sg) const {
  int first_el_count = shapes_.size();
  std::vector<std::set<int>> ret(first_el_count);
  std::vector<std::pair<int, int>> collid_list = overlap(sg);
  for (auto el : collid_list) {
    int first_val = el.first;
    if (first_val > -1 && first_val < first_el_count) {
      ret[first_val].insert(el.second);
    }
  }
  return ret;
}

/*!
 \brief Returns count of contained objects

*/

int ShapeGroup::elementCount(void) const { return shapes_.size(); }

bool ShapeGroup::rayTrace(const Eigen::Vector2d &point1,
                          const Eigen::Vector2d &point2,
                          std::vector<LineSegment> &intersect) const {
  bool res = false;
  for (auto &obj : shapes_) {
    res = obj->rayTrace(point1, point2, intersect) || res;
  }
  return res;
}

/*!
 \brief Returns Vector of pairs of indexes of the intersecting objects within
 the ShapeGroup and another ShapeGroup

 \param[in] sg - second ShapeGroup for the intersection test

*/

std::vector<std::pair<int, int>> ShapeGroup::overlap(
    const ShapeGroup &sg) const {
  const FCLCollisionObjectGroup *fcl_gr_ptr_this =
      get_fcl_object_group_ptr(this);
  const FCLCollisionObjectGroup *fcl_gr_ptr_sg = get_fcl_object_group_ptr(&sg);
  if (!fcl_gr_ptr_this || !fcl_gr_ptr_sg) {
    throw;
  }
  std::vector<std::pair<int, int>> ret;
  fcl_primitive_queries::fcl_group_overlap(*fcl_gr_ptr_this, *fcl_gr_ptr_sg,
                                           ret);
  return ret;
}

int ShapeGroup::addToIndex(const CollisionObject &obj, int idx) {
  auto el = shapes_map_.find(&obj);
  if (el != shapes_map_.end()) {
    el->second.push_back(idx);
  } else {
    std::list<int> new_list;
    new_list.push_back(idx);
    shapes_map_.emplace(&obj, new_list);
  }
  return 0;
}

int ShapeGroup::queryContainedObjectIndexList(const CollisionObject *pObj,
                                              std::list<int> &retlist) const {
  auto el = shapes_map_.find(pObj);
  if (el != shapes_map_.end()) {
    retlist.insert(retlist.end(), el->second.begin(), el->second.end());
    return 0;
  } else {
    return -1;
  }
}

void ShapeGroup::print(std::ostringstream &stream) const {}

void ShapeGroup::toString(std::ostringstream &stream) const {
  stream << "ShapeGroup "
         << "shapes: ";
  for (auto &shape : shapes_) {
    shape->toString(stream);
  }
  stream << "\\ShapeGroup " << std::endl;
}

CollisionObjectConstPtr ShapeGroup::timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return shared_ptr_this;
}

/*!
 \brief Adds a simple shape to the ShapeGroup

 The accelerator structure for collision candidates filtering is temporarily
 destroyed.

 \param[in] shape Shape to be added

  \warning The function is not thread-safe

*/

void ShapeGroup::addToGroup(ShapeConstPtr shape) {
  shapes_.push_back(shape);
  addToIndex(*shape, shapes_.size() - 1);
  invalidateCollisionEntityCache();
}

/*!
 \brief Returns Vector of all shapes contained inside the ShapeGroup

*/

std::vector<ShapeConstPtr> ShapeGroup::unpack() const { return shapes_; }

void ShapeGroup::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map,
    CollisionObjectConstPtr parent) const {
  for (auto &shape : shapes_) {
    shape->addParentMap(parent_map, parent);
  }
}

void ShapeGroup::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map) const {
  for (auto &shape : shapes_) {
    shape->addParentMap(parent_map, shared_from_this());
  }
}

std::vector<ShapeConstPtr> ShapeGroup::getShapes() { return shapes_; }
std::unordered_map<const CollisionObject *, std::list<int>> ShapeGroup::getShapesMap() { return shapes_map_; }

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::ShapeGroup &);
}

serialize::ICollisionObjectExport *ShapeGroup::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
