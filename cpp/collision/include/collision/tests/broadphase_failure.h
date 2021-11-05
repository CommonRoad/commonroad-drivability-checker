#pragma once
#include "collision/collision_object.h"
namespace collision {
namespace test {
class BroadphaseFailure {
  virtual int get_type(void) const { return 0; }
};

class BroadphaseFailureObjObj : public BroadphaseFailure {
 public:
  virtual int get_type(void) const { return 1; }
  CollisionObjectConstPtr obj1;
  CollisionObjectConstPtr obj2;
};

class BroadphaseFailureCCObj : public BroadphaseFailure {
 public:
  virtual int get_type(void) const { return 2; }
  CollisionCheckerConstPtr cc;
  CollisionObjectConstPtr obj;
};

}  // namespace test
}  // namespace collision
