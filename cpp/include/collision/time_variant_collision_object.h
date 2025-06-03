#ifndef TIME_VARIANT_COLLISION_OBJECT_H_
#define TIME_VARIANT_COLLISION_OBJECT_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "collision/collision_object.h"
#include "collision/narrowphase/shape.h"
#include "collision/shape_group.h"

namespace collision {

class TimeVariantCollisionObject;
typedef std::shared_ptr<TimeVariantCollisionObject>
    TimeVariantCollisionObjectPtr;
typedef std::shared_ptr<const TimeVariantCollisionObject>
    TimeVariantCollisionObjectConstPtr;
}  // namespace collision

namespace collision {

/*!
  \brief TimeVariantCollisionObject can contain a different CollisionObject or
  ShapeGroup at each time step


*/
class TimeVariantCollisionObject : public CollisionObjectEx {
 public:
  TimeVariantCollisionObject(int time_start_idx);
  CollisionObjectConstPtr getObstacleAtTime(int time_idx) const;
  const CollisionObject *getObstacleAtTimePtr(int time_idx) const;
  const CollisionObject *getFirstObstaclePtr() const
  {
	  return *(collision_object_at_time_ptr_.begin());
  };

  int appendObstacle(CollisionObjectConstPtr obstacle);
  virtual CollisionObjectConstPtr timeSlice(
      int time_idx, CollisionObjectConstPtr shared_ptr_this) const;

  int time_start_idx() const;
  int time_end_idx() const;

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const;

  //! Print all parameters of the shape
  virtual void print(std::ostringstream &stream) const;

  virtual void addParentMap(
      std::unordered_map<const CollisionObject *,
                         std::list<CollisionObjectConstPtr>> &parent_map) const;

  virtual CollisionObjectType getCollisionObjectType() const {
    return CollisionObjectType::OBJ_TYPE_TVOBSTACLE;
  }
  CollisionObjectClass getCollisionObjectClass() const {
    return CollisionObjectClass::OBJ_CLASS_TVOBSTACLE;
  }

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const;
#endif

 private:
  int time_start_idx_;
  int time_end_idx_;
  std::vector<CollisionObjectConstPtr> collision_object_at_time_;
  std::vector<const CollisionObject *> collision_object_at_time_ptr_;
};

typedef std::shared_ptr<TimeVariantCollisionObject>
    TimeVariantCollisionObjectPtr;
typedef std::shared_ptr<const TimeVariantCollisionObject>
    TimeVariantCollisionObjectConstPtr;

}  // namespace collision

#endif
