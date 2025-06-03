#ifndef SHAPE_GROUP_H_
#define SHAPE_GROUP_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include <set>
#include <utility>

#include "collision/application.h"
#include "collision/collision_object.h"
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/shape.h"
#include "collision/solvers/fcl/i_fcl_collision_object_group.h"

#include "collision/i_collision_container.h"

namespace collision {

typedef std::shared_ptr<ShapeGroup> ShapeGroupPtr;
typedef std::shared_ptr<const ShapeGroup> ShapeGroupConstPtr;

/**
 * \brief ShapeGroup can contain simple shapes
 *
 * The benefit of grouping objects in the ShapeGroup is that a broadphase
 * structure is built for efficient filtering of candidate objects for the
 * collision
 */

class ShapeGroup : public CollisionObjectEx,
                   public ICollisionContainer,
                   public IFCLCollisionObjectGroup {
#if ENABLE_COLLISION_TESTS == 1
  friend class collision::test::ShapeGroupTest;
#endif


 public:
  virtual const ISolverEntity_FCL *getFclInterface() const override {
    return this;
  }

  virtual FCL_COLLISION_ENTITY_TYPE getFclEntityType(void) const {
    return FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECTGROUP;
  }

  int getCollisionObjects(std::vector<fcl::CollisionObject<FCL_PRECISION> *>
                              &obj_vec) const override;

  virtual CollisionObjectType getCollisionObjectType() const {
    return CollisionObjectType::OBJ_TYPE_SHAPEGROUP;
  }
  CollisionObjectClass getCollisionObjectClass() const {
    return CollisionObjectClass::OBJ_CLASS_SHAPEGROUP;
  }

  std::shared_ptr<const collision::RectangleAABB> getAABB() const;

  ShapeGroupConstPtr windowQuery(collision::RectangleAABBConstPtr aabb) const;

  ShapeGroup() {}

  ShapeGroup(ShapeGroup &&) = default;

  void toString(std::ostringstream &stream) const;

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const;

  int elementCount(void) const;

  std::vector<std::set<int>> overlapMap(const ShapeGroup &sg) const;
  std::vector<std::pair<int, int>> overlap(const ShapeGroup &sg) const;

  virtual void print(std::ostringstream &stream) const;
  virtual CollisionObjectConstPtr timeSlice(
      int time_idx, CollisionObjectConstPtr shared_ptr_this) const;

  void addToGroup(ShapeConstPtr shape);
  std::vector<ShapeConstPtr> unpack() const;
#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const;
#endif

  const ICollisionContainer *getContainerInterface(void) const override {
    return this;
  }

  virtual void addParentMap(
      std::unordered_map<const CollisionObject *,
                         std::list<CollisionObjectConstPtr>> &parent_map) const;
  virtual void addParentMap(
      std::unordered_map<const CollisionObject *,
                         std::list<CollisionObjectConstPtr>> &parent_map,
      CollisionObjectConstPtr parent) const;

  int queryContainedObjectIndexList(const CollisionObject *,
                                    std::list<int> &retlist) const override;

protected:
    std::vector<ShapeConstPtr> getShapes();
    std::unordered_map<const CollisionObject *, std::list<int>> getShapesMap();

 private:
  std::vector<ShapeConstPtr> shapes_;
  std::unordered_map<const CollisionObject *, std::list<int>> shapes_map_;

  int addToIndex(const CollisionObject &, int idx);
};

}  // namespace collision

#endif
