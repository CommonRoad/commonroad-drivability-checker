#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "collision/narrowphase/polygon.h"
#include "collision/serialize/collision_object_export_s11.h"
#include "collision/serialize/export_structs/polygon_export_struct.h"

namespace collision {
namespace serialize {
class PolygonExport : public IShapeExport {
 public:
  PolygonExport() {}
  PolygonExport(const Polygon &sg);
  virtual ~PolygonExport();
  CollisionObject *loadObject(void);

  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  std::vector<ICollisionObjectExport_s11 *> m_triangles;
  std::vector<Eigen::Vector2d> m_vertices;
  std::vector<std::vector<Eigen::Vector2d>> m_hole_vertices;
  PolygonExportStruct m_fields;
};
}  // namespace serialize

}  // namespace collision
