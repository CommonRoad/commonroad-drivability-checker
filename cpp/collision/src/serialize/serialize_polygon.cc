#include "collision/application_settings.h"
#if ENABLE_SERIALIZER

#include <Eigen/Dense>
#include <istream>

#include "collision/serialize/vector2d_export_streams.h" // redefines the stream operators for the module

#include "collision/serialize/serialize.h"

#include "collision/narrowphase/polygon.h"
#include "collision/serialize/vector2d_export.h"

#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace collision {
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Polygon &polygon) {
  return new PolygonExport(polygon);
}
PolygonExport::PolygonExport(const Polygon &polyg) {
  m_fields.center_x = polyg.center_x();
  m_fields.center_y = polyg.center_y();

  for (auto &obj : polyg.getTriangleMesh()) {
    // warning: unsafe cast
    m_triangles.push_back(static_cast<TriangleExport *>(obj->exportThis()));
  }

  m_vertices = polyg.getVertices();
  m_hole_vertices = polyg.getHoleVertices();
}

PolygonExport::~PolygonExport() {
  for (auto obj : m_triangles) {
    if (obj) delete obj;
  }
}
bool PolygonExport::operator()(s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "PolygonExport");
  for (auto el : m_triangles) {
    if (!el) return false;
  }
  bool res = true;
  res = res && s11n::list::serialize_list(dest, "triangles", m_triangles);
  res = res && s11n::list::serialize_list(dest, "vertices", m_vertices);
  res =
      res && s11n::list::serialize_list(dest, "hole_vertices", m_hole_vertices);

  return res;
}
bool PolygonExport::operator()(const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  bool res = true;
  res =
      res && s11n::list::deserialize_list(src, "triangles", this->m_triangles);
  res = res && s11n::list::deserialize_list(src, "vertices", this->m_vertices);
  res = res && s11n::list::deserialize_list(src, "hole_vertices",
                                            this->m_hole_vertices);
  return res;
}

CollisionObject *PolygonExport::loadObject(void) {
  // load all triangles without memory leaks
  // then create the polygon using internal structures
  // std::vector<Eigen::Vector2d> vertices;
  // std::vector<std::vector<Eigen::Vector2d> > hole_vertices;
  std::vector<TriangleConstPtr> mesh_triangles;

  for (auto obj : m_triangles) {
    CollisionObject *loaded_obj_ptr = obj->loadObject();
    if (!loaded_obj_ptr) {
      return nullptr;
    }
    CollisionObjectConstPtr loaded_obj(loaded_obj_ptr);
    if (loaded_obj->getCollisionObjectClass() != OBJ_CLASS_SHAPE) {
      return nullptr;
    }
    if ((static_cast<const Shape *>(loaded_obj.get()))->type() !=
        ShapeType::TYPE_TRIANGLE) {
      return nullptr;
    }

    mesh_triangles.push_back(
        std::static_pointer_cast<const Triangle>(loaded_obj));
  }

  Polygon *polyg =
      new Polygon(m_vertices, m_hole_vertices, mesh_triangles,
                  Eigen::Vector2d(m_fields.center_x, m_fields.center_y));
  return polyg;
}
}  // namespace serialize
}  // namespace collision
#endif
