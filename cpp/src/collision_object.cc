#include "collision/collision_object.h"

#if ENABLE_SERIALIZER
#include "collision/serialize/public/serialize_public.h"
#endif

namespace collision {

#if ENABLE_SERIALIZER
int CollisionObject::serialize(std::ostream &output_stream) const {
  return serialize::serialize(*this, output_stream);
}

CollisionObjectConstPtr CollisionObject::deserialize(
    std::istream &input_stream) {
  CollisionObjectConstPtr ret;
  if (!serialize::deserialize(ret, input_stream)) {
    return ret;
  } else
    return CollisionObjectConstPtr(0);
}
#endif

void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(shared_from_this());
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(shared_from_this());
    parent_map.emplace(this, newlist);
  }
}
void CollisionObject::addParentMap(
    std::unordered_map<const CollisionObject *,
                       std::list<CollisionObjectConstPtr>> &parent_map,
    CollisionObjectConstPtr parent) const {
  auto it = parent_map.find((const CollisionObject *)this);
  if (it != parent_map.end()) {
    it->second.push_back(parent);
  } else {
    auto newlist = std::list<CollisionObjectConstPtr>();
    newlist.push_back(parent);
    parent_map.emplace(this, newlist);
  }
}

}  // namespace collision
