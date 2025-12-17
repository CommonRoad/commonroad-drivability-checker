#include "collision/solvers/fcl/fcl_collision_object.h"
#include <math.h>
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/solvers/fcl/fcl_helpers.h"

namespace collision {
namespace solvers {
namespace solverFCL {

collision::RectangleAABBConstPtr FCLCollisionObject::getAABB() const {
  fcl::CollisionObject<FCL_PRECISION> *fcl_this_obj =
      this->getCollisionObject_fcl().get();
  fcl::AABB<double> BV_this = fcl_this_obj->getAABB();

  Eigen::Vector2d min_(BV_this.min_[0], BV_this.min_[1]);

  Eigen::Vector2d max_(BV_this.max_[0], BV_this.max_[1]);

  Eigen::Vector2d center = (min_ + max_) / 2;

  double rx = center[0] - min_[0];
  double ry = center[1] - min_[1];

  collision::RectangleAABBConstPtr raabbcptr =
      std::shared_ptr<const RectangleAABB>(
          new collision::RectangleAABB(rx, ry, center));
  return raabbcptr;
}

bool FCLCollisionObject::BVCheck(CollisionObjectConstPtr obj2) const {
  SolverEntity_FCL *obj2_entity;
  FCL_COLLISION_ENTITY_TYPE obj2_entity_type =
      get_object_fcl_entity_type(obj2, obj2_entity);

  if (obj2_entity_type ==
      FCL_COLLISION_ENTITY_TYPE::COLLISION_ENTITY_TYPE_FCL_OBJECT) {
    fcl::CollisionObject<FCL_PRECISION> *fcl_this_obj =
        this->getCollisionObject_fcl().get();
    fcl::CollisionObject<FCL_PRECISION> *fcl_obj2 =
        (static_cast<const FCLCollisionObject *>(obj2_entity))
            ->getCollisionObject_fcl()
            .get();
    fcl::AABB<double> BV_this = fcl_this_obj->getAABB();
    fcl::AABB<double> BV_2 = fcl_obj2->getAABB();
    return BV_this.overlap(BV_2);
  }
  return true;
}

std::shared_ptr<fcl::CollisionObject<FCL_PRECISION>>
FCLCollisionObject::getCollisionObject_fcl(void) const {
  if (!object_cached)

  {
    fcl_geometry = std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>>(
        m_parent_fcl_interface->createFCLCollisionGeometry());

    fcl_col_obj = std::shared_ptr<fcl::CollisionObject<FCL_PRECISION>>(
        m_parent_fcl_interface->createFCLCollisionObject(fcl_geometry));

    // TODO: alloc errors check

    fcl::CollisionObject<FCL_PRECISION> *fcl_obj = fcl_col_obj.get();

    fcl_obj->setUserData((void *)m_parent);

    object_cached = true;
  }
  return (fcl_col_obj);
}

}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision
