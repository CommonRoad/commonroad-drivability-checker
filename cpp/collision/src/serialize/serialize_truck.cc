#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/truck.h"

#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Truck &tr) {
  return new TruckExport(tr);
}
TruckExport::TruckExport(const Truck &tr) {
  for (auto &obj : tr.unpack()) {
    // warning: unsafe typecast
    m_children.push_back(static_cast<IShapeExport *>(obj->exportThis()));
  }
}

TruckExport::~TruckExport() {
  for (auto obj : m_children) {
    if (obj) delete obj;
  }
}
bool TruckExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "TruckExport");
  for (auto el : m_children) {
    if (!el) return false;
  }
  s11n::list::serialize_list(dest, "children", m_children);
  return true;
}
bool TruckExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  return s11n::list::deserialize_list(src, "children", this->m_children);
}

CollisionObject *TruckExport::loadObject(void) {
  auto tr = new Truck();
  for (auto obj : m_children) {
    CollisionObject *loaded_obj_ptr = obj->loadObject();
    if (!loaded_obj_ptr) {
      delete tr;
      return nullptr;
    }
    CollisionObjectConstPtr loaded_obj(loaded_obj_ptr);
    if (loaded_obj->getCollisionObjectClass() != OBJ_CLASS_SHAPE) {
      delete tr;
      return nullptr;
    }
    tr->addToGroup(std::static_pointer_cast<const Shape>(loaded_obj));
  }
  return tr;
}
}  // namespace serialize
}  // namespace collision
#endif
