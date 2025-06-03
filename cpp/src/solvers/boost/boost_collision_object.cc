#include "collision/solvers/boost/boost_collision_object.h"

#include "collision/solvers/boost/boost_object_polygon.h"

namespace collision {
namespace solvers {
namespace solverBoost {

std::shared_ptr<BoostObjectInternal>
BoostCollisionObject::getCollisionObject_boost(void) const {
  if (!object_cached)

  {
    switch (m_parent->getCollisionObjectType())

    {
      case CollisionObjectType::OBJ_TYPE_POLYGON:
        boost_obj = std::static_pointer_cast<BoostObjectInternal>(
            std::make_shared<BoostPolygon>(
                static_cast<const collision::Polygon*>(m_parent)));
        break;

      default:
        throw 0;
    }

    object_cached = true;
  }
  return (boost_obj);
}

}  // namespace solverBoost
}  // namespace solvers
}  // namespace collision
