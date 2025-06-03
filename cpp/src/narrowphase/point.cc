#include "collision/narrowphase/point.h"
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

namespace collision {

fcl::CollisionGeometry<FCL_PRECISION> *Point::createFCLCollisionGeometry(
    void) const {
  return new fcl::Sphere<FCL_PRECISION>(COLLISION_FCL_POINT_EPS);
}
fcl::CollisionObject<FCL_PRECISION> *Point::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom,
      collision::FCLTransform::fcl_get_3d_translation(this->center()));
}

Point *Point::clone() const { return new Point(*this); }

Point::Point(const Point &copy) : Shape(copy) {}

void Point::print(std::ostringstream &stream) const {
  stream << "Point: center: (" << center_x() << "/" << center_y() << ")"
         << std::endl;
}

ShapeType Point::type() const { return type_; }

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::Point &);
}

serialize::ICollisionObjectExport *Point::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
