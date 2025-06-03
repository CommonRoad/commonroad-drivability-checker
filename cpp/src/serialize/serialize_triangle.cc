#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include "collision/narrowphase/triangle.h"
namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Triangle &triangle) {
  return new TriangleExport(triangle);
}

TriangleExport::TriangleExport(const Triangle &triangle) {
  m_fields.v1_x = triangle.v1()[0];
  m_fields.v1_y = triangle.v1()[1];

  m_fields.v2_x = triangle.v2()[0];
  m_fields.v2_y = triangle.v2()[1];

  m_fields.v3_x = triangle.v3()[0];
  m_fields.v3_y = triangle.v3()[1];
}

CollisionObject *TriangleExport::loadObject(void) {
  return new Triangle(Eigen::Vector2d(m_fields.v1_x, m_fields.v1_y),
                      Eigen::Vector2d(m_fields.v2_x, m_fields.v2_y),
                      Eigen::Vector2d(m_fields.v3_x, m_fields.v3_y));
}

bool TriangleExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "TriangleExport");
  TR::set(dest, "v1_x", m_fields.v1_x);
  TR::set(dest, "v1_y", m_fields.v1_y);

  TR::set(dest, "v2_x", m_fields.v2_x);
  TR::set(dest, "v2_y", m_fields.v2_y);

  TR::set(dest, "v3_x", m_fields.v3_x);
  TR::set(dest, "v3_y", m_fields.v3_y);

  return true;
}
bool TriangleExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.v1_x = TR::get(src, "v1_x", double(0));
  m_fields.v1_y = TR::get(src, "v1_y", double(0));

  m_fields.v2_x = TR::get(src, "v2_x", double(0));
  m_fields.v2_y = TR::get(src, "v2_y", double(0));

  m_fields.v3_x = TR::get(src, "v3_x", double(0));
  m_fields.v3_y = TR::get(src, "v3_y", double(0));

  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
