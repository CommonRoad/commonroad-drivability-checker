
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/narrowphase/rectangle_aabb.h"
namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::RectangleAABB &aabb) {
  return new RectangleAABBExport(aabb);
}

RectangleAABBExport::RectangleAABBExport(const RectangleAABB &rect) {
  m_fields.rx = rect.r_x();
  m_fields.ry = rect.r_y();
  m_fields.center_x = rect.center()[0];
  m_fields.center_y = rect.center()[1];
}

CollisionObject *RectangleAABBExport::loadObject(void) {
  return new RectangleAABB(
      m_fields.rx, m_fields.ry,
      Eigen::Vector2d(m_fields.center_x, m_fields.center_y));
}

bool RectangleAABBExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "RectangleAABBExport");
  TR::set(dest, "rx", m_fields.rx);
  TR::set(dest, "ry", m_fields.ry);
  TR::set(dest, "cx", m_fields.center_x);
  TR::set(dest, "cy", m_fields.center_y);

  return true;
}
bool RectangleAABBExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.rx = TR::get(src, "rx", double(0));
  m_fields.ry = TR::get(src, "ry", double(0));
  m_fields.center_x = TR::get(src, "cx", double(0));
  m_fields.center_y = TR::get(src, "cy", double(0));
  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
