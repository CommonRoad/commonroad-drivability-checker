#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include "collision/serialize/final/collision_object_export_final.h"
namespace collision {
namespace serialize {
bool CollisionObjectExport_final_s11::operator()(
    s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "CollisionObjectExport_final_s11");
  return s11nlite::serialize_subnode(dest, "base", base);
}

bool CollisionObjectExport_final_s11::operator()(
    const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;

  const s11nlite::node_type *ch = s11n::find_child_by_name(src, "base");
  if (!ch) {
    // CERR << "Deser of 'base' member failed: node not found!\n";
    return false;
  }
  base = s11nlite::deserialize<ICollisionObjectExport_s11>(*ch);
  return 0 != base;
}

}  // namespace serialize
}  // namespace collision

#endif
