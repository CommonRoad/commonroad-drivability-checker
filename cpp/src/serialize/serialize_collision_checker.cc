
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>
#include "collision/collision_checker.h"

namespace collision {
namespace serialize {
ICollisionCheckerExport *exportObject(const collision::CollisionChecker &cc) {
  return new CollisionCheckerExport(cc);
}
CollisionCheckerExport::CollisionCheckerExport(const CollisionChecker &cc) {
  for (auto &obj : cc.getObstacles()) {
    // warning: unsafe typecast
    m_children.push_back(
        static_cast<ICollisionObjectExport_s11 *>(obj->exportThis()));
  }
}

CollisionCheckerExport::~CollisionCheckerExport() {
  for (auto obj : m_children) {
    if (obj) delete obj;
  }
}
bool CollisionCheckerExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "CollisionCheckerExport");
  for (auto el : m_children) {
    if (!el) return false;
  }
  return s11n::list::serialize_list(dest, "children", m_children);
}
bool CollisionCheckerExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  bool res = s11n::list::deserialize_list(src, "children", this->m_children);
  return res;
}

CollisionChecker *CollisionCheckerExport::loadObject(void) {
  CollisionChecker *cc = new CollisionChecker();
  for (auto obj : m_children) {
    CollisionObject *loaded_obj_ptr = obj->loadObject();
    if (!loaded_obj_ptr) {
      delete cc;
      return nullptr;
    }
    CollisionObjectConstPtr loaded_obj(loaded_obj_ptr);
    cc->addCollisionObject(loaded_obj);
  }
  return cc;
}
}  // namespace serialize
}  // namespace collision
#endif
