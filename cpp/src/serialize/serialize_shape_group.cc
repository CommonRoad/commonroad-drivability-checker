#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/shape_group.h"

#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::ShapeGroup &sg) {
  return new ShapeGroupExport(sg);
}
ShapeGroupExport::ShapeGroupExport(const ShapeGroup &sg) {
  for (auto &obj : sg.unpack()) {
    // warning: unsafe typecast
    m_children.push_back(static_cast<IShapeExport *>(obj->exportThis()));
  }
}

ShapeGroupExport::~ShapeGroupExport() {
  for (auto obj : m_children) {
    if (obj) delete obj;
  }
}
bool ShapeGroupExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "ShapeGroupExport");
  for (auto el : m_children) {
    if (!el) return false;
  }
  s11n::list::serialize_list(dest, "children", m_children);
  return true;
}
bool ShapeGroupExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  return s11n::list::deserialize_list(src, "children", this->m_children);
}

CollisionObject *ShapeGroupExport::loadObject(void) {
  ShapeGroup *sg = new ShapeGroup();
  for (auto obj : m_children) {
    CollisionObject *loaded_obj_ptr = obj->loadObject();
    if (!loaded_obj_ptr) {
      delete sg;
      return nullptr;
    }
    CollisionObjectConstPtr loaded_obj(loaded_obj_ptr);
    if (loaded_obj->getCollisionObjectClass() != OBJ_CLASS_SHAPE) {
      delete sg;
      return nullptr;
    }
    sg->addToGroup(std::static_pointer_cast<const Shape>(loaded_obj));
  }
  return sg;
}
}  // namespace serialize
}  // namespace collision
#endif
