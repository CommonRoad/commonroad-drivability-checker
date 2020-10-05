#pragma once

namespace collision {
class CollisionObject;
namespace serialize {
class ICollisionObjectExport {
 public:
  ICollisionObjectExport() {}
  virtual CollisionObject *loadObject(void) { return nullptr; };
  virtual ~ICollisionObjectExport(){};
};
}  // namespace serialize
}  // namespace collision
