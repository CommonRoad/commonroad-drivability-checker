#include "collision/time_variant_collision_object.h"
#include "collision/solvers/geometry_queries.h"

namespace collision {

/*!
 \brief A helper function that is called from the rayTracePrimitive function.
 Given the query line segment [point1, point2], it outputs the part(s) of the line segment that intersect with the contained Shapes (at all time steps).
 If the contained object is a ShapeGroup, ShapeGroup::rayTrace is called.
 \param[in] point1 - start of the query line segment
 \param[in] point2 - end of the query line segment
 \param[out] intersect - vector to which the output line segments are to be appended
*/

bool TimeVariantCollisionObject::rayTrace(
    const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
    std::vector<LineSegment> &intersect) const {
  bool res = false;
  for (auto &obj : collision_object_at_time_) {
    res = obj->rayTrace(point1, point2, intersect) || res;
  }
  return res;
}

/*!
 \brief Prints out important information about the TimeVariantCollisionObject
 \param[out] stream - output stringstream to print the information to
*/

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

/*!
 \brief Retrieves a shared pointer to the contained Obstacle (Shape or ShapeGroup) at the given time step
 \param[in] time_idx - input time index
*/

CollisionObjectConstPtr TimeVariantCollisionObject::getObstacleAtTime(
    int time_idx) const {
  if (time_idx < time_start_idx_ || time_idx > time_end_idx_) return nullptr;
  return collision_object_at_time_[time_idx - time_start_idx_];
}

/*!
 \brief Retrieves a pointer to the contained obstacle (Shape or ShapeGroup) at the given time step
 \param[in] time_idx - input time index
*/

const CollisionObject *TimeVariantCollisionObject::getObstacleAtTimePtr(
    int time_idx) const {
  if (time_idx < time_start_idx_ || time_idx > time_end_idx_) return nullptr;
  return collision_object_at_time_ptr_[time_idx - time_start_idx_];
}

/*!
 \brief getter for time_start_idx_ - index for the first time step in the TimeVariantCollisionObject
*/

int TimeVariantCollisionObject::time_start_idx() const {
  return time_start_idx_;
}

/*!
 \brief getter for time_end_idx_ - index for the last time step in the TimeVariantCollisionObject
*/

int TimeVariantCollisionObject::time_end_idx() const { return time_end_idx_; }

/*!
 \brief Helper function used in CollisionChecker::timeSlice that returns the obstacle (Shape or ShapeGroup) at the given time-step index
 \param[in] time_idx - input time-step index
 \param[in] shared_ptr_this [unused] shared pointer to the TimeVariantCollisionObject itself
*/

CollisionObjectConstPtr TimeVariantCollisionObject::timeSlice(
    int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
  return getObstacleAtTime(time_idx);
}

/*!
 \brief adds an obstacle at the next time step
 \param[in] obstacle - input obstacle (Shape or ShapeGroup) to add at the next time step
*/

int TimeVariantCollisionObject::appendObstacle(
    CollisionObjectConstPtr obstacle) {
  collision_object_at_time_.push_back(obstacle);
  collision_object_at_time_ptr_.push_back(obstacle.get());
  return ++time_end_idx_;
}

/*!
 \brief Function that is called from FCLCollisionChecker::setUpParentMap. FCLCollisionChecker needs to be able to find which container object
 (e.g. ShapeGroup or TimeVariantCollisionObject) contains the simple Shape that collides with the given object in order to return all
 containing obstacles. To do this, it keeps track of these container objects in the parent_map data structure. It is an unordered map that contains
 the list of all containers immediately added to the CollisionChecker that contain the simple shape.
 This function calls the addParentMap member function of each of the CollisionObjects contained within this TimeVariantCollisionObject
 (CollisionObject::addParentMap or ShapeGroup::addParentMap) which in turn adds the shared pointer to this TimeVariantCollisionObject
 into the parent_map data structure for each of the simple Shapes contained within this TimeVariantCollisionObject.
 As the result, shared pointer to this TimeVariantCollisionObject is added to the list of parents for all the simple Shapes that it contains.
  \param[in] parent_map reference to the parent_map data structure of FCLCollisionChecker
*/

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

/*!
 \brief Exports the TimeVariantCollisionObject into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *TimeVariantCollisionObject::exportThis(
    void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
