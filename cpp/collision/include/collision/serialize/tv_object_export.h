#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/collision_object_export_s11.h"
#include "collision/serialize/export_structs/tv_object_export_struct.h"
#include "collision/time_variant_collision_object.h"

namespace collision {
namespace serialize {

class TimeVariantCollisionObjectExport : public ICollisionObjectExport_s11 {
 public:
  TimeVariantCollisionObjectExport() {}
  TimeVariantCollisionObjectExport(const TimeVariantCollisionObject &sg);
  virtual ~TimeVariantCollisionObjectExport();
  CollisionObject *loadObject(void);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  std::vector<ICollisionObjectExport_s11 *> m_children;
  TimeVariantCollisionObjectExportStruct m_fields;
};
}  // namespace serialize

}  // namespace collision
