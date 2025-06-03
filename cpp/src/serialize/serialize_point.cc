
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/narrowphase/point.h"
namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Point &point) {
  return new PointExport(point);
}

PointExport::PointExport(const Point &point) {
  m_fields.center_x = point.center_x();
  m_fields.center_y = point.center_y();
}

CollisionObject *PointExport::loadObject(void) {
  return new Point(Eigen::Vector2d(m_fields.center_x, m_fields.center_y));
}

bool PointExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "PointExport");
  TR::set(dest, "center_x", m_fields.center_x);
  TR::set(dest, "center_y", m_fields.center_y);

  return true;
}
bool PointExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.center_x = TR::get(src, "center_x", double(0));

  m_fields.center_y = TR::get(src, "center_y", double(0));

  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
