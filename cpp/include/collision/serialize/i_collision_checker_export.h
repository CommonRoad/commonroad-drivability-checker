#pragma once

namespace collision {
class CollisionChecker;
namespace serialize {
class ICollisionCheckerExport {
 public:
  ICollisionCheckerExport() {}
  virtual CollisionChecker *loadObject(void) { return nullptr; };
  virtual ~ICollisionCheckerExport(){};
};
}  // namespace serialize
}  // namespace collision
