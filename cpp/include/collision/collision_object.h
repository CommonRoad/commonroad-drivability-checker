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

#include "collision/i_collision_container.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"

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

using namespace solvers::solverFCL;

/*!
  \brief Base class for CollisionObjects and some of their groups

*/
class CollisionObject : public std::enable_shared_from_this<CollisionObject> {
 public:
  
  CollisionObject() {};
  virtual ~CollisionObject() {};
  CollisionObject(CollisionObject&& other) {
    // when the object is moved, all pointers to solvers (e.g. fcl_entity) are to become invalid
      other.fcl_entity_.reset();
      other.fcl_solver_entity_valid_ = false;
  }
  
  CollisionObject& operator=(CollisionObject&& other) {
    // when the object is moved, all pointers to solvers (e.g. fcl_entity) are to become invalid
    if (this != &other) {
      other.fcl_entity_.reset();
      other.fcl_solver_entity_valid_ = false;
    }
    return *this;
  }
  CollisionObject(const CollisionObject&) = delete;
  CollisionObject& operator=(const CollisionObject&) = delete;
  
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

  virtual void print(std::ostringstream &stream) const { return; }

  virtual void toString(std::ostringstream &stream) const { print(stream); }

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

  virtual bool collide(
        const CollisionObject &c,
        const collision::CollisionRequest &req = CollisionRequest()) const;

    virtual bool BVCheck(CollisionObjectConstPtr obj2) const;

    virtual std::shared_ptr<const collision::RectangleAABB> getAABB() const;

    virtual int getSolverEntity(solvers::solverFCL::SolverEntity_FCL *&ptr) const;

    virtual const ICollisionContainer *getContainerInterface(void) const {
      return nullptr;
    }

    virtual const solvers::solverFCL::ISolverEntity_FCL *getFclInterface(
        void) const {
      return nullptr;
    }
    virtual bool is_valid() const { return true;}
  protected:
    void invalidateCollisionEntityCache(void);

  private:
    mutable std::unique_ptr<solvers::solverFCL::SolverEntity_FCL>
        fcl_entity_;
    mutable bool fcl_solver_entity_valid_ = false;
};

typedef std::shared_ptr<Shape> ShapePtr;
typedef std::shared_ptr<const Shape> ShapeConstPtr;

typedef std::shared_ptr<RectangleAABB> RectangleAABBPtr;
typedef std::shared_ptr<const RectangleAABB> RectangleAABBConstPtr;

typedef std::shared_ptr<RectangleOBB> RectangleOBBPtr;
typedef std::shared_ptr<const RectangleOBB> RectangleOBBConstPtr;

}  // namespace collision

#endif
