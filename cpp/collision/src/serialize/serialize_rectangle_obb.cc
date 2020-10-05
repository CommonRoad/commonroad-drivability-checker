
#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER
#include "collision/narrowphase/rectangle_obb.h"
namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::RectangleOBB &obb) {
  return new RectangleOBBExport(obb);
}

RectangleOBBExport::RectangleOBBExport(const RectangleOBB &rect) {
  m_fields.local_axis_x_1 = rect.local_x_axis()[0];
  m_fields.local_axis_x_2 = rect.local_x_axis()[1];
  m_fields.local_axis_y_1 = rect.local_y_axis()[0];
  m_fields.local_axis_y_2 = rect.local_y_axis()[1];
  m_fields.rx = rect.r_x();
  m_fields.ry = rect.r_y();
  m_fields.center_x = rect.center()[0];
  m_fields.center_y = rect.center()[1];
}

CollisionObject *RectangleOBBExport::loadObject(void) {
  Eigen::Matrix2d local_axes;
  local_axes.col(0)[0] = m_fields.local_axis_x_1;
  local_axes.col(0)[1] = m_fields.local_axis_x_2;
  local_axes.col(1)[0] = m_fields.local_axis_y_1;
  local_axes.col(1)[1] = m_fields.local_axis_y_2;

  return new RectangleOBB(
      m_fields.rx, m_fields.ry, local_axes,
      Eigen::Vector2d(m_fields.center_x, m_fields.center_y));
}

bool RectangleOBBExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "RectangleOBBExport");
  TR::set(dest, "rx", m_fields.rx);
  TR::set(dest, "ry", m_fields.ry);
  TR::set(dest, "local_axis_x_1", m_fields.local_axis_x_1);
  TR::set(dest, "local_axis_x_2", m_fields.local_axis_x_2);
  TR::set(dest, "local_axis_y_1", m_fields.local_axis_y_1);
  TR::set(dest, "local_axis_y_2", m_fields.local_axis_y_2);
  TR::set(dest, "cx", m_fields.center_x);
  TR::set(dest, "cy", m_fields.center_y);

  return true;
}
bool RectangleOBBExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.rx = TR::get(src, "rx", double(0));
  m_fields.ry = TR::get(src, "ry", double(0));
  m_fields.local_axis_x_1 = TR::get(src, "local_axis_x_1", double(0));
  m_fields.local_axis_x_2 = TR::get(src, "local_axis_x_2", double(0));
  m_fields.local_axis_y_1 = TR::get(src, "local_axis_y_1", double(0));
  m_fields.local_axis_y_2 = TR::get(src, "local_axis_y_2", double(0));
  m_fields.center_x = TR::get(src, "cx", double(0));
  m_fields.center_y = TR::get(src, "cy", double(0));
  return true;
}
}  // namespace serialize
}  // namespace collision
#endif
