#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include "collision/time_variant_collision_object.h"

#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(
    const collision::TimeVariantCollisionObject &sg) {
  return new TimeVariantCollisionObjectExport(sg);
}
TimeVariantCollisionObjectExport::TimeVariantCollisionObjectExport(
    const TimeVariantCollisionObject &tvobst) {
  m_fields.time_start_idx = tvobst.time_start_idx();
  for (int time_idx = m_fields.time_start_idx;
       time_idx <= tvobst.time_end_idx(); time_idx++) {
    auto obj = tvobst.getObstacleAtTime(time_idx);
    if (!obj.get()) {
      m_children.push_back(nullptr);
    }
    // warning: unsafe typecast
    m_children.push_back(
        static_cast<ICollisionObjectExport_s11 *>(obj->exportThis()));
  }
}

TimeVariantCollisionObjectExport::~TimeVariantCollisionObjectExport() {
  for (auto &obj : m_children) {
    if (obj) delete obj;
  }
}
bool TimeVariantCollisionObjectExport::operator()(
    s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "TimeVariantCollisionObjectExport");
  for (auto el : m_children) {
    if (!el) return false;
  }
  bool res = true;
  res = res && s11n::list::serialize_list(dest, "children", m_children);
  TR::set(dest, "time_start_idx", m_fields.time_start_idx);
  return res;
}
bool TimeVariantCollisionObjectExport::operator()(
    const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  bool res = s11n::list::deserialize_list(src, "children", this->m_children);
  m_fields.time_start_idx = TR::get(src, "time_start_idx", double(0));
  return res;
}

CollisionObject *TimeVariantCollisionObjectExport::loadObject(void) {
  TimeVariantCollisionObject *tvobj =
      new TimeVariantCollisionObject(m_fields.time_start_idx);
  for (auto &obj : m_children) {
    CollisionObject *loaded_obj_ptr = obj->loadObject();
    if (!loaded_obj_ptr) {
      delete tvobj;
      return nullptr;
    }
    CollisionObjectConstPtr loaded_obj(loaded_obj_ptr);
    if (loaded_obj->getCollisionObjectClass() == OBJ_CLASS_SHAPE) {
      tvobj->appendObstacle(std::static_pointer_cast<const Shape>(loaded_obj));
    } else if (loaded_obj->getCollisionObjectClass() == OBJ_CLASS_SHAPEGROUP) {
      tvobj->appendObstacle(
          std::static_pointer_cast<const ShapeGroup>(loaded_obj));
    } else {
      delete tvobj;
      return nullptr;
    }
  }
  return tvobj;
}
}  // namespace serialize
}  // namespace collision
#endif
