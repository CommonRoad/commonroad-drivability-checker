#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/narrowphase/sphere.h"
namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Sphere &sphere) {
  return new SphereExport(sphere);
}

SphereExport::SphereExport(const Sphere &sphere) {
  m_fields.radius = sphere.radius();

  m_fields.center_x = sphere.center_x();
  m_fields.center_y = sphere.center_y();
}

CollisionObject *SphereExport::loadObject(void) {
  return new Sphere(m_fields.radius,
                    Eigen::Vector2d(m_fields.center_x, m_fields.center_y));
}

bool SphereExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "SphereExport");
  TR::set(dest, "radius", m_fields.radius);
  TR::set(dest, "center_x", m_fields.center_x);
  TR::set(dest, "center_y", m_fields.center_y);

  return true;
}
bool SphereExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.radius = TR::get(src, "radius", double(0));
  m_fields.center_x = TR::get(src, "center_x", double(0));

  m_fields.center_y = TR::get(src, "center_y", double(0));

  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
