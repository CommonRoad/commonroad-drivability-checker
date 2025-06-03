#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/narrowphase/point.h"
#include "collision/serialize/export_structs/point_export_struct.h"
#include "collision/serialize/shape_export.h"

namespace collision {
namespace serialize {
class PointExport : public IShapeExport {
 public:
  PointExport() {}
  PointExport(const Point &rect);
  CollisionObject *loadObject(void);
  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  PointExportStruct m_fields;
};

}  // namespace serialize

}  // namespace collision
