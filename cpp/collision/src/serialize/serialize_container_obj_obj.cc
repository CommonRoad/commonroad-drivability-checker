
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace serialize {
BroadphaseFailure_obj_objExport::BroadphaseFailure_obj_objExport(void) {}
BroadphaseFailure_obj_objExport::BroadphaseFailure_obj_objExport(
    const test::BroadphaseFailureObjObj &object) {
  m_object = object;
}
BroadphaseFailure_obj_objExport::~BroadphaseFailure_obj_objExport(void) {}
const test::BroadphaseFailure *BroadphaseFailure_obj_objExport::getFailure(
    void) const {
  return &m_object;
}

bool BroadphaseFailure_obj_objExport::operator()(
    s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "BroadphaseFailure_obj_objExport");
  if (!m_object.obj1.get() || !m_object.obj2.get()) {
    return false;
  }
  std::shared_ptr<ICollisionObjectExport_s11> obj1_exp(
      static_cast<ICollisionObjectExport_s11 *>(m_object.obj1->exportThis()));
  std::shared_ptr<ICollisionObjectExport_s11> obj2_exp(
      static_cast<ICollisionObjectExport_s11 *>(m_object.obj2->exportThis()));
  if (!obj1_exp.get() || !obj2_exp.get()) {
    return false;
  }
  bool res = true;
  res = res && s11nlite::serialize_subnode(dest, "obj1", obj1_exp.get());
  res = res && s11nlite::serialize_subnode(dest, "obj2", obj2_exp.get());

  return res;
}
bool BroadphaseFailure_obj_objExport::operator()(
    const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;

  const s11nlite::node_type *obj1_node = s11n::find_child_by_name(src, "obj1");
  const s11nlite::node_type *obj2_node = s11n::find_child_by_name(src, "obj2");
  if (!obj1_node || !obj2_node) {
    // CERR << "Deser of 'base' member failed: node not found!\n";
    return false;
  }

  std::shared_ptr<ICollisionObjectExport_s11> obj1_imp(
      s11nlite::deserialize<ICollisionObjectExport_s11>(*obj1_node));
  std::shared_ptr<ICollisionObjectExport_s11> obj2_imp(
      s11nlite::deserialize<ICollisionObjectExport_s11>(*obj2_node));

  if (!obj1_imp.get() || !obj2_imp.get()) {
    return false;
  }
  collision::CollisionObjectConstPtr obj1_ptr(obj1_imp.get()->loadObject());
  collision::CollisionObjectConstPtr obj2_ptr(obj2_imp.get()->loadObject());

  if (!obj1_ptr.get() || !obj2_ptr.get()) {
    return false;
  }

  m_object.obj1 = obj1_ptr;
  m_object.obj2 = obj2_ptr;

  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
