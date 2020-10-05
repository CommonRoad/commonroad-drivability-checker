#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/serialize/collision_object_export_s11.h"

namespace collision {
namespace serialize {
class IShapeExport : public ICollisionObjectExport_s11 {};

}  // namespace serialize

}  // namespace collision
