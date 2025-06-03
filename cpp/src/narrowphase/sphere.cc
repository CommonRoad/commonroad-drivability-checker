
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"

#include "collision/narrowphase/point.h"

#include "collision/raytrace_primitive.h"

#include <fcl/math/constants.h>

#include "collision/narrowphase/sphere.h"

namespace collision {

fcl::CollisionGeometry<FCL_PRECISION> *Sphere::createFCLCollisionGeometry(
    void) const {
  return new fcl::Sphere<FCL_PRECISION>(this->radius());
}

fcl::CollisionObject<FCL_PRECISION> *Sphere::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(
      col_geom,
      collision::FCLTransform::fcl_get_3d_translation(this->center()));
}

bool Sphere::rayTrace(const Eigen::Vector2d &point1,
                      const Eigen::Vector2d &point2,
                      std::vector<LineSegment> &intersect) const {
  // LineSegment obj_segment(point1, point2);

  std::vector<Eigen::Vector2d> inters1;
  int res = 0;
  res = raytrace::findLineCircleIntersections(this->center_x(),
                                              this->center_y(), this->radius(),
                                              point1, point2, inters1);
  if (res == 2) {
    raytrace::Point p1(point1);
    raytrace::Point p2(point2);
    raytrace::Point p_int1(inters1[0]);
    raytrace::Point p_int2(inters1[1]);

    bool o1 = raytrace::onSegment(p1, p_int1, p2);
    bool o2 = raytrace::onSegment(p1, p_int2, p2);
    if (o1 && o2) {
      intersect.push_back(LineSegment(inters1[0], inters1[1]));
    } else {
      if (this->collide(collision::Point(point1),
                        CollisionRequest(COL_DEFAULT))) {
        if (o1) {
          intersect.push_back(LineSegment(inters1[0], point1));
        } else if (o2) {
          intersect.push_back(LineSegment(inters1[1], point1));
        } else {
          intersect.push_back(LineSegment(point1, point2));
        }
      } else if (o1 || o2) {
        if (o1) {
          intersect.push_back(LineSegment(inters1[0], point2));
        } else {
          intersect.push_back(LineSegment(inters1[1], point2));
        }

      } else {
        intersect.push_back(LineSegment(point1, point2));
      }
    }

  } else if (res == 1) {
    if (this->collide(collision::Point(point1),
                      CollisionRequest(COL_DEFAULT))) {
      intersect.push_back(LineSegment(inters1[0], point1));
    } else {
      intersect.push_back(LineSegment(inters1[0], point2));
    }
  }

  // collision::raytrace::raytrace_postprocess(point1, point2, inters1,
  // intersect, this );

  return res;

  return false;
}

Sphere *Sphere::clone() const { return new Sphere(*this); }

Sphere::Sphere(const Sphere &copy) : Shape(copy) {
  center_ = copy.center();
  radius_ = copy.radius();
}

void Sphere::print(std::ostringstream &stream) const {
  stream << "Sphere:\n"
         << "center: (" << center_x() << "|" << center_y() << ")\n"
         << "radius: " << radius_ << std::endl;
}

void Sphere::set_radius(double _radius) {
  radius_ = _radius;
  invalidateCollisionEntityCache();
}

ShapeType Sphere::type() const { return type_; }

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::Sphere &);
}

serialize::ICollisionObjectExport *Sphere::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
