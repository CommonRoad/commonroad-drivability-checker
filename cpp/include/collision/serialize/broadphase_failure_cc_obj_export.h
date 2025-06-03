#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/serialize.h"
#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace serialize {
class BroadphaseFailure_cc_objExport : public BroadphaseFailureExport {
 public:
  BroadphaseFailure_cc_objExport(void);
  BroadphaseFailure_cc_objExport(const test::BroadphaseFailureCCObj &object);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

  virtual ~BroadphaseFailure_cc_objExport(void);
  const test::BroadphaseFailure *getFailure(void) const;

 protected:
  test::BroadphaseFailureCCObj m_object;
};

}  // namespace serialize

}  // namespace collision
