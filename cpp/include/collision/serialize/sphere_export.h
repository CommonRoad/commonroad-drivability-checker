#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/narrowphase/sphere.h"
#include "collision/serialize/collision_object_export_s11.h"
#include "collision/serialize/export_structs/sphere_export_struct.h"

namespace collision {
namespace serialize {
class SphereExport : public IShapeExport {
 public:
  SphereExport() {}
  SphereExport(const Sphere &rect);
  CollisionObject *loadObject(void);
  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  SphereExportStruct m_fields;
};

}  // namespace serialize

}  // namespace collision
