#include "collision/primitive_collision_checker.h"

namespace collision {

void PrimitiveCollisionChecker::addCollisionObject(CollisionObjectConstPtr co) {
  collision_objects_.push_back(co);
}

bool PrimitiveCollisionChecker::collide(CollisionObjectConstPtr co) const {
  for (auto &c : collision_objects_) {
    if (c->BVCheck(co)) {
      if (c->collide(*co, CollisionRequest(COL_DEFAULT))) {
        return true;
      }
    }
  }
  return false;
}

bool PrimitiveCollisionChecker::collide(
    CollisionObjectConstPtr co, CollisionObjectConstPtr &obstacle) const {
  for (auto &c : collision_objects_) {
    if (c->BVCheck(co)) {
      if (c->collide(*co, CollisionRequest(COL_DEFAULT))) {
        obstacle = c;
        return true;
      }
    }
  }
  return false;
}

bool PrimitiveCollisionChecker::collide(
    CollisionObjectConstPtr co,
    std::vector<CollisionObjectConstPtr> &obstacles) const {
  bool collides = false;
  for (auto &c : collision_objects_) {
    if (c->BVCheck(co)) {
      if (c->collide(*co, CollisionRequest(COL_DEFAULT))) {
        obstacles.push_back(c);
        collides = true;
      }
    }
  }
  return collides;
}

PrimitiveCollisionCheckerPtr PrimitiveCollisionChecker::windowQuery(
    const RectangleAABB &aabb) const {
  PrimitiveCollisionCheckerPtr cc_ret =
      std::shared_ptr<PrimitiveCollisionChecker>(
          new PrimitiveCollisionChecker());
  for (auto &c : collision_objects_) {
    if (c->collide(aabb, CollisionRequest(COL_DEFAULT))) {
      cc_ret->addCollisionObject(c);
    }
  }
  return cc_ret;
}

PrimitiveCollisionCheckerPtr PrimitiveCollisionChecker::timeSlice(
    int time_idx) const {
  PrimitiveCollisionCheckerPtr cc_ret(new PrimitiveCollisionChecker());
  for (auto &c : collision_objects_) {
    CollisionObjectConstPtr tmp = c->timeSlice(time_idx, c);
    if (tmp != nullptr) {
      cc_ret->addCollisionObject(tmp);
    }
  }
  return cc_ret;
}

int PrimitiveCollisionChecker::numberOfObstacles() const {
  return collision_objects_.size();
}

std::vector<CollisionObjectConstPtr> PrimitiveCollisionChecker::getObstacles()
    const {
  return collision_objects_;
}

void PrimitiveCollisionChecker::print(std::ostringstream &stream) const {
  stream << "PrimitiveCollisionChecker number of CollisionObjects: "
         << collision_objects_.size() << std::endl;
  for (unsigned int i = 0; i < collision_objects_.size(); i++) {
    collision_objects_[i]->print(stream);
  }
}

}  // namespace collision
