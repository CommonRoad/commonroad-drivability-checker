#include "collision/collision_checker.h"

#include "collision/narrowphase/rectangle_obb.h"
#include "collision/primitive_collision_checker.h"
#include "collision/raytrace_utils.h"
#if ENABLE_SERIALIZER
#include "collision/serialize/public/serialize_public.h"
#endif

namespace collision {

CollisionCheckerPtr CollisionChecker::clone_shared(void) const {
  CollisionCheckerPtr new_cc =
      CollisionCheckerPtr(new collision::CollisionChecker());
  for (auto el : collision_objects_) {
    new_cc->addCollisionObject(el);
  }
  return new_cc;
}

/*!
 \brief Adds a collision object into the CollisionChecker group structure

*/
void CollisionChecker::addCollisionObject(CollisionObjectConstPtr co) {
  collision_objects_.push_back(co);
  fcl_cc_.addCollisionObject(co);
}

/*!
 \brief Returns timestep at which any of the objects contained in the
 CollisionChecker collides with the given object

 The function returns -1 if there is no collision or timestep of the collision
 is not defined (such as for static obstacles)


 \warning The function is not thread-safe

*/

int CollisionChecker::collisionTime(CollisionObjectConstPtr co) const {
  int res = -1;
  collide(co, &res);
  return res;
}

/*!
 \brief Returns true if any of the objects contained in the CollisionChecker
 collide with the given object

 \param[in] co given object

 \param[out] optional output pointer for getting the time of the collision


 \warning The function is not thread-safe

*/
#if (ENABLE_COLLISION_TESTS)
bool CollisionChecker::collide(CollisionObjectConstPtr co, int *collision_time,
                               bool enable_test) const {
  if (enable_test) {
    collision::test::CollisionCheckerTest cc_test;
    cc_test.run_test_collide(co, this);
  }
#else
bool CollisionChecker::collide(CollisionObjectConstPtr co,
                               int *collision_time) const {

#endif

  // STACK_TIMER tim(TIMER_collide);
  return fcl_cc_.collideHelper(co, collision_time, 0);

  bool collides = false;
}

/*!
 \brief Returns true if any of the objects contained in the CollisionChecker
collide with the given object

 \param[in] co given object
 \param[in] ungroup_shape_groups when false, ShapeGroups will be treated as
separate objects and included in the result. Otherwise, one of the colliding
obstacles within the ShapeGroups will be added to the output Vector.

 \param[in] ungroup_TV_obstacles when false, TimeVarintCollisionObjects will be
treated as separate objects and included in the result. Otherwise, one of the
colliding obstacles within the TimeVarintCollisionObject will be added to the
output Vector.

 \param[out] obstacle output reference for the colliding object


 \warning The function is not thread-safe

*/

#if (ENABLE_COLLISION_TESTS)
bool CollisionChecker::collide(CollisionObjectConstPtr co,
                               CollisionObjectConstPtr &obstacle,
                               bool ungroup_shape_groups,
                               bool ungroup_TV_obstacles,
                               bool enable_test) const {
  if (enable_test) {
    std::cout << "testing";
    collision::test::CollisionCheckerTest cc_test;
    cc_test.run_test_collide_obstacle(co, this);
  }
#else
bool CollisionChecker::collide(CollisionObjectConstPtr co,
                               CollisionObjectConstPtr &obstacle,
                               bool ungroup_shape_groups,
                               bool ungroup_TV_obstacles) const {

#endif

  std::vector<CollisionObjectConstPtr> obstacles;
  bool res = fcl_cc_.collideHelper(co, 0, &obstacles, 1, ungroup_shape_groups,
                                   ungroup_TV_obstacles);
  if (res) {
    if (obstacles.size() > 0) {
      obstacle = obstacles[0];
    }
  }
  return res;
}

/*!
 \brief Performs raytracing for a given line segment.
 The function returns true if the line segment between the given two points
 collides with any object inside the CollisionChecker. In addition, the function
 returns Vector with the parts of the given line segment that belong to the
 occupied space.

 The function finds all intersection points between the given line segment and
 all line segments or circles that the colliding objects are made of.

 If the given line segment start or end point lies within an object, the line
 segment between this point inside the object and the point of collision is
 added.

 If remove_overlaps is set to true, the function joins together all overlapping
 intervals for the final result.

 \param[in] point1 line segment begin point

 \param[in] point2 line segment end point

 \param[in] remove_overlaps if true, the the function joins together all
 overlapping intervals for the final result.

 \warning The function is not thread-safe


*/

bool CollisionChecker::rayTrace(const Eigen::Vector2d &point1,
                                const Eigen::Vector2d &point2,
                                std::vector<LineSegment> &intersect,
                                bool remove_overlaps) {
  double eps = 0.000001;
  double sqrlen =
      (pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
  // if ray length is greater than epsilon, filter the colliding obstacles
  if (sqrlen >= eps) {
    Eigen::Vector2d center = (point2 + point1) / 2;
    Eigen::Vector2d local_x_axis = center - point1;

    double rad_x = sqrt(sqrlen) / 2;  // radius of the ray OBB
    local_x_axis /= rad_x;
    Eigen::Vector2d local_y_axis(-local_x_axis[1], local_x_axis[0]);
    Eigen::Matrix2d local_axes;
    local_axes.col(0) = local_x_axis;
    local_axes.col(1) = local_y_axis;
    collision::RectangleOBBConstPtr robb(
        new collision::RectangleOBB(rad_x, eps, local_axes, center));
    std::vector<CollisionObjectConstPtr> obstacles;

    if (collide(robb, obstacles, 1, 1)) {
      return collision::raytrace::rayTracePrimitive(obstacles, point1, point2,
                                                    intersect, remove_overlaps);
    } else
      return false;

  } else {
    return collision::raytrace::rayTracePrimitive(
        collision_objects_, point1, point2, intersect, remove_overlaps);
  }
}

void CollisionChecker::toString(std::ostringstream &stream) const {
  stream << "CollisionChecker "
         << "objects: ";

  for (auto &obj : collision_objects_) {
    obj->toString(stream);
  }
  stream << "\\CollisionChecker " << std::endl;
}

/*!
 \brief Returns true if any of the objects contained in the CollisionChecker
 collide with the given object

 \param[in] co given object
 \param[in] ungroup_shape_groups when false, ShapeGroups will be treated as
 separate objects and included in the result. Otherwise, the colliding obstacles
 within the ShapeGroups will be added to the output Vector.

 \param[in] ungroup_TV_obstacles when false, TimeVariantCollisionObjects will be
 treated as separate objects and included in the result. Otherwise, the
 colliding obstacles within the TimeVariantCollisionObject will be added to the
 output Vector.

 \param[out] obstacles output Vector for colliding objects


 \warning The function is not thread-safe

*/

#if (ENABLE_COLLISION_TESTS)
bool CollisionChecker::collide(CollisionObjectConstPtr co,
                               std::vector<CollisionObjectConstPtr> &obstacles,
                               bool ungroup_shape_groups,
                               bool ungroup_TV_obstacles,
                               bool enable_test) const {
  if (enable_test) {
    collision::test::CollisionCheckerTest cc_test;
    cc_test.run_test_collide_obstacles(co, this);
  }
#else
bool CollisionChecker::collide(CollisionObjectConstPtr co,
                               std::vector<CollisionObjectConstPtr> &obstacles,
                               bool ungroup_shape_groups,
                               bool ungroup_TV_obstacles) const {
#endif

  return fcl_cc_.collideHelper(co, 0, &obstacles, -1, ungroup_shape_groups,
                               ungroup_TV_obstacles);

  bool collides = false;
  for (auto &c : collision_objects_)
    if (c->collide(*co)) {
      obstacles.push_back(c);
      collides = true;
    }
  return collides;
}

/*!
 \brief Returns new PrimitiveCollisionChecker with all static objects within the
 window and all time-variant obstacles. Ungroups all shape groups.

 \param[in] aabb - reference to a axis-aligned rectangle representing the region
 of interest

 \warning The function is not thread-safe

*/

PrimitiveCollisionCheckerPtr CollisionChecker::windowQueryPrimitive(
    const RectangleAABB &aabb) const {
  PrimitiveCollisionCheckerPtr cc_ret =
      std::make_shared<PrimitiveCollisionChecker>();

  fcl_cc_.windowQuery_helper(aabb, *(cc_ret.get()));

  return cc_ret;
}

/*!
 \brief Returns new CollisionChecker with all static objects within the window
 and all time-variant obstacles. Ungroups all shape groups.

 \param[in] aabb - reference to an axis-aligned rectangle representing the
 region of interest

 \warning The function is not thread-safe


*/
CollisionCheckerPtr CollisionChecker::windowQuery(
    const RectangleAABB &aabb) const {
  // STACK_TIMER tim(TIMER_windowQuery);

  CollisionCheckerPtr cc_ret = std::make_shared<CollisionChecker>();
  fcl_cc_.windowQuery_helper(aabb, *(cc_ret.get()));

  return cc_ret;
}

/*!
 \brief Returns new CollisionChecker with objects that exist at a given timestep

 \param[in] time_idx - index of timestep of interest

*/

CollisionCheckerPtr CollisionChecker::timeSlice(int time_idx) const {
  // STACK_TIMER tim(TIMER_timeSlice);
  CollisionCheckerPtr cc_ret(new CollisionChecker());
  for (auto &c : collision_objects_) {
    CollisionObjectConstPtr tmp = c->timeSlice(time_idx, c);
    if (tmp != nullptr) {
      cc_ret->addCollisionObject(tmp);
    }
  }
  return cc_ret;
}

/*!
 \brief Returns number of contained CollisionObjects of any type

*/

int CollisionChecker::numberOfObstacles() const {
  return collision_objects_.size();
}

/*!
 \brief Returns a Vector with all contained CollisionObjects of any type

*/

std::vector<CollisionObjectConstPtr> CollisionChecker::getObstacles() const {
  return collision_objects_;
}

void CollisionChecker::print(std::ostringstream &stream) const {
  stream << "CollisionChecker number of CollisionObjects: "
         << collision_objects_.size() << std::endl;
  for (unsigned int i = 0; i < collision_objects_.size(); i++) {
    collision_objects_[i]->print(stream);
  }
}

#if ENABLE_SERIALIZER

int CollisionChecker::serialize(std::ostream &output_stream) const {
  return serialize::serialize(*this, output_stream);
}

CollisionCheckerConstPtr CollisionChecker::deserialize(
    std::istream &input_stream) {
  CollisionCheckerConstPtr ret;
  if (!serialize::deserialize(ret, input_stream)) {
    return ret;
  } else
    return CollisionCheckerConstPtr(0);
}

namespace serialize {
ICollisionCheckerExport *exportObject(const collision::CollisionChecker &);
}

serialize::ICollisionCheckerExport *CollisionChecker::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
