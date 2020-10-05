#ifndef COLLISION_OBJECT_H_
#define COLLISION_OBJECT_H_

#include <iostream>
#include <list>
#include <memory>
#include <sstream>
#include <unordered_map>

#include <map>

#include "collision/application_settings.h"

#if ENABLE_SERIALIZER
#include "collision/i_collision_object_export.h"
#endif

#include "collision/solvers/collision_requests.h"

#include <Eigen/Dense>

#include "collision/collision_object_types.h"
#include "collision/line_segment.h"

namespace collision {

class Shape;
class Point;
class RectangleAABB;
class RectangleOBB;
class Sphere;
class Triangle;
class TimeVariantCollisionObject;
class CollisionObject;
class ShapeGroup;
class Polygon;

class ICollisionContainer;

typedef std::shared_ptr<CollisionObject> CollisionObjectPtr;
typedef std::shared_ptr<const CollisionObject> CollisionObjectConstPtr;

/*!
  \brief Base class for CollisionObjects and some of their groups

*/
class CollisionObject : public std::enable_shared_from_this<CollisionObject> {
 public:
  virtual ~CollisionObject() {}
#if ENABLE_SERIALIZER
  virtual serialize::ICollisionObjectExport *exportThis(void) const {
    return nullptr;
  }

  int serialize(std::ostream &output_stream) const;
  static CollisionObjectConstPtr deserialize(std::istream &output_stream);

#endif

  virtual CollisionObjectType getCollisionObjectType() const {
    return CollisionObjectType::OBJ_TYPE_UNKNOWN;
  }
  virtual CollisionObjectClass getCollisionObjectClass() const {
    return CollisionObjectClass::OBJ_CLASS_UNKNOWN;
  }

  virtual bool collide(
      const CollisionObject &c,
      const collision::CollisionRequest &req = CollisionRequest()) const = 0;

  virtual void print(std::ostringstream &stream) const { return; }

  virtual void toString(std::ostringstream &stream) const { print(stream); }

  virtual bool BVCheck(CollisionObjectConstPtr obj2) const = 0;

  virtual std::shared_ptr<const collision::RectangleAABB> getAABB() const = 0;

  virtual void addParentMap(
      std::unordered_map<const CollisionObject *,
                         std::list<CollisionObjectConstPtr>> &parent_map) const;
  virtual void addParentMap(
      std::unordered_map<const CollisionObject *,
                         std::list<CollisionObjectConstPtr>> &parent_map,
      CollisionObjectConstPtr parent) const;

  virtual CollisionObjectConstPtr timeSlice(
      int time_idx, CollisionObjectConstPtr shared_ptr_this) const = 0;

  virtual bool rayTrace(const Eigen::Vector2d &point1,
                        const Eigen::Vector2d &point2,
                        std::vector<LineSegment> &intersect) const {
    return false;
  }
};

typedef std::shared_ptr<Shape> ShapePtr;
typedef std::shared_ptr<const Shape> ShapeConstPtr;

typedef std::shared_ptr<RectangleAABB> RectangleAABBPtr;
typedef std::shared_ptr<const RectangleAABB> RectangleAABBConstPtr;

typedef std::shared_ptr<RectangleOBB> RectangleOBBPtr;
typedef std::shared_ptr<const RectangleOBB> RectangleOBBConstPtr;

}  // namespace collision

#endif
