#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/narrowphase/triangle.h"
#include "collision/serialize/collision_object_export_s11.h"
#include "collision/serialize/export_structs/triangle_export_struct.h"

namespace collision {
namespace serialize {
class TriangleExport : public IShapeExport {
 public:
  TriangleExport() {}
  TriangleExport(const Triangle &rect);
  CollisionObject *loadObject(void);
  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  TriangleExportStruct m_fields;
};

}  // namespace serialize

}  // namespace collision
