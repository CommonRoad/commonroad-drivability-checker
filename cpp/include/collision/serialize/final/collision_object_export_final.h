#pragma once
#include "collision/serialize/collision_object_export_s11.h"
namespace collision {
namespace serialize {
class CollisionObjectExport_final_s11 {
  // warning: memory for the member base must be deallocated manually upon each
  // deserialization and construction of this class
 public:
  ICollisionObjectExport_s11 *base;
  CollisionObjectExport_final_s11(ICollisionObjectExport_s11 *_base)
      : base(_base) {}
  CollisionObjectExport_final_s11() {}
  virtual ~CollisionObjectExport_final_s11() {}
  virtual bool operator()(s11nlite::node_type &dest) const;

  virtual bool operator()(const s11nlite::node_type &src);
};
}  // namespace serialize
}  // namespace collision
