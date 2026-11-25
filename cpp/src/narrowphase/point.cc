#include "collision/narrowphase/point.h"
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

namespace collision {

/*!
 \brief creates FCL collision geometry for a Point. This is a FCL library internal representation used for collision checking.
*/

fcl::CollisionGeometry<FCL_PRECISION> *Point::createFCLCollisionGeometry(
    void) const {
  return new fcl::Sphere<FCL_PRECISION>(COLLISION_FCL_POINT_EPS);
}

/*!
 \brief creates FCL collision object for a Point. This is a FCL library internal representation used for collision checking.
 \param[in] col_geom - corresponding FCL collision geometry
*/

fcl::CollisionObject<FCL_PRECISION> *Point::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom,
      collision::FCLTransform::fcl_get_3d_translation(this->center()));
}

/*!
 \brief Clones the Point
*/

Point *Point::clone() const { return new Point(*this); }

/*!
 \brief Copy constructor for a Point
*/

Point::Point(const Point &copy) : Shape(copy) {}

/*!
 \brief Prints out important information about the Point
 \param[out] stream - output stringstream to print the information to
*/

void Point::print(std::ostringstream &stream) const {
  stream << "Point: center: (" << center_x() << "/" << center_y() << ")"
         << std::endl;
}

/*!
 \brief Returns the type of the Shape
*/

ShapeType Point::type() const { return type_; }

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::Point &);
}

/*!
 \brief Exports the Point into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *Point::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
