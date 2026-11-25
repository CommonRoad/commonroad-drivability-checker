#include "collision/collision_object.h"

#if ENABLE_SERIALIZER
#include "collision/serialize/public/serialize_public.h"
#endif

#include "collision/solvers/collision_queries.h"
#include "collision/solvers/fcl/i_solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl.h"
#include "collision/solvers/fcl/solver_entity_fcl_factory.h"

namespace collision {

#if ENABLE_SERIALIZER

/*!
 \brief Serializes the CollisionObject into an output stream
 \param[out] output_stream - the output stream to serialize the object into
*/

int CollisionObject::serialize(std::ostream &output_stream) const {
  return serialize::serialize(*this, output_stream);
}

/*!
 \brief Deserializes the CollisionObject from an input stream
 \param[in] input_stream - the input stream to deserialize the object from
*/

CollisionObjectConstPtr CollisionObject::deserialize(
    std::istream &input_stream) {
  CollisionObjectConstPtr ret;
  if (!serialize::deserialize(ret, input_stream)) {
    return ret;
  } else
    return CollisionObjectConstPtr(0);
}
#endif

/*!
 \brief Function that is called from FCLCollisionChecker::setUpParentMap. FCLCollisionChecker needs to be able to find which container object
 (e.g. ShapeGroup or TimeVariantCollisionObject) contains the simple Shape that collides with the given object in order to return all
 containing obstacles. To do this, it keeps track of these container objects in the parent_map data structure. It is an unordered map that contains
 the list of all containers immediately added to the CollisionChecker that contain the simple shape.
 This function adds the shared pointer to the simple shape itself into the parent_map. It has the meaning that the simple shape was directly
 added into the CollisionChecker.
 The function is overriden by ShapeGroup::addParentMap and TimeVariantCollisionObject::addParentMap.
  \param[in] parent_map reference to the parent_map data structure of FCLCollisionChecker
*/

void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(shared_from_this());
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(shared_from_this());
    parent_map.emplace(this, newlist);
  }
}

/*!
 \brief Function that is called from ShapeGroup::addParentMap and TimeVariantCollisionObject::addParentMap.
 FCLCollisionChecker needs to be able to find which container object (e.g. ShapeGroup or TimeVariantCollisionObject) contains
 the simple Shape that collides with the given object in order to return all containing obstacles.
 To do this, it keeps track of these container objects in the parent_map data structure. It is an unordered map that contains
 the list of all containers immediately added to the CollisionChecker that contain the simple shape.
 This function adds the provided parent shared pointer (see input parameters) into the parent_map. It has the meaning that the outermost parent
 (directly added into the CollisionChecker) is added into the parent_map for the simple Shape.
 The function is overriden by ShapeGroup::addParentMap.
  \param[in] parent_map reference to the parent_map data structure of FCLCollisionChecker
  \param[in] parent the outermost parent of the simple shape directly added into the CollisionChecker
*/

void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map,
    CollisionObjectConstPtr parent) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(parent);
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(parent);
    parent_map.emplace(this, newlist);
  }
}

/*!
 \brief Returns whether the CollisionObject collides with another CollisionObject.
 \param[in] c - the other CollisionObject
 \param[in] req - parameters for the collision request
*/

bool CollisionObject::collide(const CollisionObject &c,
                                const collision::CollisionRequest &req) const {
  CollisionResult res;
  collision::collide_binary(*this, c, res, req);
  return res.collides();
}

/*!
 \brief Returns whether the axis-aligned bounding box for this CollisionObject collides with the axis-aligned bounding box
 for the other CollisionObject.
 \param[in] obj2 - the other CollisionObject
*/

bool CollisionObject::BVCheck(CollisionObjectConstPtr obj2) const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->BVCheck(obj2);
  }
  return true;
}

/*!
 \brief Gets the axis-aligned bounding box for this CollisionObject. It works only for simple Shapes.
 Uses FCL library representation to compute the AABB or gets the cached AABB (previously computed using FCL representation).
*/

std::shared_ptr<const collision::RectangleAABB> CollisionObject::getAABB()
    const {
  SolverEntity_FCL *entity;
  getSolverEntity(entity);
  if (entity && entity->getFclEntityType() ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    return (static_cast<FCLCollisionObject *>(entity))->getAABB();
  }
  std::cout
      << "Returning bounding volume for complex shapes is not implemented";
  throw;
}

/*!
 \brief Each CollisionObject contains a unique pointer to the fcl_entity. It is the FCL collision library representation for this CollisionObject.
It has a meaning for simple Shapes and ShapeGroup. This function is to be called whenever the CollisionObject changes in a way that
requires recomputation of the cached FCL collision library representation.
*/

void CollisionObject::invalidateCollisionEntityCache(void) {
  if (fcl_solver_entity_valid_) {
    fcl_entity_->invalidateSolverEntityCache();
  }
}

/*!
 \brief Getter for the contained fcl_entity_ FCL collision library representation.
 fcl_entity_ is recomputed whenever it is flagged to be invalid (using the invalidateCollisionEntityCache function).
*/

int CollisionObject::getSolverEntity(SolverEntity_FCL *&ptr) const {
  SolverEntity_FCL *cur_value = fcl_entity_.get();
  if (fcl_solver_entity_valid_) {
    ptr = cur_value;
    return 0;
  } else {
    // FCL
    fcl_entity_ =
        std::move(std::unique_ptr<SolverEntity_FCL>(
            createFCLSolverEntity(this)));
    fcl_solver_entity_valid_ = true;
    ptr = fcl_entity_.get();
    return 0;
  }
}
}  // namespace collision
