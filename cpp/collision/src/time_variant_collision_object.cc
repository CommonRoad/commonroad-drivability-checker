#include "collision/time_variant_collision_object.h"
#include "collision/solvers/geometry_queries.h"

namespace collision {

bool TimeVariantCollisionObject::rayTrace(
    const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
    std::vector<LineSegment> &intersect) const {
  bool res = false;
  for (auto &obj : collision_object_at_time_) {
    res = obj->rayTrace(point1, point2, intersect) || res;
  }
  return res;
}

void TimeVariantCollisionObject::print(std::ostringstream &stream) const {
  stream << "Timevariant obstacle, time " << time_start_idx_ << "-"
         << time_end_idx_ << std::endl;
  for (unsigned int i = 0; i < collision_object_at_time_.size(); i++) {
    stream << "  " << i + time_start_idx_ << ":";
    collision_object_at_time_[i]->print(stream);
  }
}

/*!
 \brief Creates a new TimeVariantCollisionObject

 \param[in] time_start_idx index of the first timestep

*/
TimeVariantCollisionObject::TimeVariantCollisionObject(int time_start_idx) {
  time_start_idx_ = time_start_idx;
  time_end_idx_ = time_start_idx - 1;
}

CollisionObjectConstPtr TimeVariantCollisionObject::getObstacleAtTime(
    int time_idx) const {
  if (time_idx < time_start_idx_ || time_idx > time_end_idx_) return nullptr;
  return collision_object_at_time_[time_idx - time_start_idx_];
}

const CollisionObject *TimeVariantCollisionObject::getObstacleAtTimePtr(
    int time_idx) const {
  if (time_idx < time_start_idx_ || time_idx > time_end_idx_) return nullptr;
  return collision_object_at_time_ptr_[time_idx - time_start_idx_];
}

int TimeVariantCollisionObject::time_start_idx() const {
  return time_start_idx_;
}

int TimeVariantCollisionObject::time_end_idx() const { return time_end_idx_; }

CollisionObjectConstPtr TimeVariantCollisionObject::timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return getObstacleAtTime(time_idx);
}

int TimeVariantCollisionObject::appendObstacle(
    CollisionObjectConstPtr obstacle) {
  collision_object_at_time_.push_back(obstacle);
  collision_object_at_time_ptr_.push_back(obstacle.get());
  return ++time_end_idx_;
}

void TimeVariantCollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map) const {
  for (auto &tvobj : collision_object_at_time_) {
    tvobj->addParentMap(parent_map, shared_from_this());
  }
  return;
}

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(
    const collision::TimeVariantCollisionObject &);
}

serialize::ICollisionObjectExport *TimeVariantCollisionObject::exportThis(
    void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
