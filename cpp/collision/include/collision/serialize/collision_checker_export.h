#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/collision_checker.h"
#include "collision/i_collision_checker_export.h"

#include "collision/serialize/collision_object_export_s11.h"

namespace collision {
namespace serialize {
class CollisionCheckerExport : public ICollisionCheckerExport {
 public:
  CollisionCheckerExport() {}
  CollisionCheckerExport(const CollisionChecker &cc);
  virtual ~CollisionCheckerExport();
  CollisionChecker *loadObject(void);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  std::vector<ICollisionObjectExport_s11 *> m_children;
};
}  // namespace serialize

}  // namespace collision
