#pragma once
#include <s11n.net/s11n/s11nlite.hpp>
#include "collision/i_collision_object_export.h"

namespace collision {
class CollisionObject;
namespace serialize {
class ICollisionObjectExport_s11 : public ICollisionObjectExport {
 public:
  ICollisionObjectExport_s11(void) {}
  virtual bool operator()(s11nlite::node_type &dest) const { return true; }
  virtual bool operator()(const s11nlite::node_type &src) { return true; }

  virtual ~ICollisionObjectExport_s11(void){};
};
}  // namespace serialize

}  // namespace collision
