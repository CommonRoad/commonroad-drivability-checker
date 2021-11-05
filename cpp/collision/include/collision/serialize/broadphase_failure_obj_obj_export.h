#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/serialize.h"
#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace serialize {
class BroadphaseFailure_obj_objExport : public BroadphaseFailureExport {
 public:
  BroadphaseFailure_obj_objExport(void);
  BroadphaseFailure_obj_objExport(const test::BroadphaseFailureObjObj &object);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

  virtual ~BroadphaseFailure_obj_objExport(void);
  const test::BroadphaseFailure *getFailure(void) const;

 protected:
  test::BroadphaseFailureObjObj m_object;
};

}  // namespace serialize

}  // namespace collision
