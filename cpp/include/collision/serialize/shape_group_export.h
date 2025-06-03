#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/collision_object_export_s11.h"
#include "collision/shape_group.h"

namespace collision {
namespace serialize {
class ShapeGroupExport : public ICollisionObjectExport_s11 {
 public:
  ShapeGroupExport() {}
  ShapeGroupExport(const ShapeGroup &sg);
  virtual ~ShapeGroupExport();
  CollisionObject *loadObject(void);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  std::vector<ICollisionObjectExport_s11 *> m_children;
};
}  // namespace serialize

}  // namespace collision
