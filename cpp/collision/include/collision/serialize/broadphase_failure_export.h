#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/collision_object_export_s11.h"

namespace collision {
namespace serialize {
class BroadphaseFailureExport {
 public:
  BroadphaseFailureExport(void) {}
  virtual bool operator()(s11nlite::node_type &dest) const { return true; }
  virtual bool operator()(const s11nlite::node_type &src) { return true; }
  virtual ~BroadphaseFailureExport(void) {}
};
}  // namespace serialize

}  // namespace collision
