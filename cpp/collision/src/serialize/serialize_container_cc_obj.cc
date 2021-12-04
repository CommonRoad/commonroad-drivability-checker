
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace serialize {
BroadphaseFailure_cc_objExport::BroadphaseFailure_cc_objExport(void) {}
BroadphaseFailure_cc_objExport::BroadphaseFailure_cc_objExport(
    const test::BroadphaseFailureCCObj &object) {
  m_object = object;
}
BroadphaseFailure_cc_objExport::~BroadphaseFailure_cc_objExport(void) {}
const test::BroadphaseFailure *BroadphaseFailure_cc_objExport::getFailure(
    void) const {
  return &m_object;
}

bool BroadphaseFailure_cc_objExport::operator()(
    s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "BroadphaseFailure_cc_objExport");
  if (!m_object.cc.get() || !m_object.obj.get()) {
    return false;
  }
  std::shared_ptr<CollisionCheckerExport> cc_exp(
      static_cast<CollisionCheckerExport *>(m_object.cc->exportThis()));
  std::shared_ptr<ICollisionObjectExport_s11> obj_exp(
      static_cast<ICollisionObjectExport_s11 *>(m_object.obj->exportThis()));
  if (!cc_exp.get() || !obj_exp.get()) {
    return false;
  }
  bool res = true;
  res = res && s11nlite::serialize_subnode(dest, "cc", cc_exp.get());
  res = res && s11nlite::serialize_subnode(dest, "obj", obj_exp.get());

  return res;
}
bool BroadphaseFailure_cc_objExport::operator()(
    const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;

  const s11nlite::node_type *cc_node = s11n::find_child_by_name(src, "cc");
  const s11nlite::node_type *obj_node = s11n::find_child_by_name(src, "obj");
  if (!cc_node || !obj_node) {
    // CERR << "Deser of 'base' member failed: node not found!\n";
    return false;
  }

  std::shared_ptr<CollisionCheckerExport> cc_exp(
      s11nlite::deserialize<CollisionCheckerExport>(*cc_node));
  std::shared_ptr<ICollisionObjectExport_s11> obj_exp(
      s11nlite::deserialize<ICollisionObjectExport_s11>(*obj_node));

  if (!cc_exp.get() || !obj_exp.get()) {
    return false;
  }
  collision::CollisionCheckerConstPtr cc_ptr(cc_exp.get()->loadObject());
  collision::CollisionObjectConstPtr obj_ptr(obj_exp.get()->loadObject());

  if (!cc_ptr.get() || !obj_ptr.get()) {
    return false;
  }

  m_object.cc = cc_ptr;
  m_object.obj = obj_ptr;

  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
