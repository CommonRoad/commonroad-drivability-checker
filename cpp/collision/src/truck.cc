#include "collision/truck.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/utils.h"
#include "collision/solvers/fcl/fcl_collision_queries.h"
#include "collision/solvers/fcl/fcl_helpers.h"

namespace collision {

    CollisionObjectConstPtr Truck::timeSlice(int time_idx, CollisionObjectConstPtr shared_ptr_this) const {
        return shared_ptr_this;
    }

    int Truck::queryContainedObjectIndexList(const CollisionObject *pObj, std::list<int> &retlist) const {
        auto shapes_map = shapeGroup_->getShapesMap();
        auto el = shapes_map.find(pObj);
        if (el != shapes_map.end()) {
            retlist.insert(retlist.end(), el->second.begin(), el->second.end());
            return 0;
        }
        return -1;
    }

    int Truck::getCollisionObjects(std::vector<fcl::CollisionObject<FCL_PRECISION> *> &obj_vec) const {
        auto shapes = shapeGroup_->getShapes();
        for (auto &i : shapes) {
            const FCLCollisionObject *i_obj = get_fcl_object_ptr(i.get());
            fcl::CollisionObject<FCL_PRECISION> *obj = i_obj->getCollisionObject_fcl().get();  // TODO: thread-safety
            if (!obj) return 1;
            obj_vec.push_back(obj);
        }
        return 0;
    }

    int Truck::elementCount(void) const { return shapeGroup_->getShapes().size(); }

    void Truck::init()
    {
        double head_length = (2 * length_ - 2 * trailer_length_ - trailer_dist_) / 2;
        Eigen::Vector2d head_center = center_ + Eigen::Vector2d(abs(length_ - head_length), width_ * sin(orientation_));
        RectangleOBB head = RectangleOBB(head_length, width_, orientation_, head_center);

        Eigen::Vector2d trailer_center = center_ - Eigen::Vector2d(abs(length_ - trailer_length_), -sin(orientation_) / width_ );
        RectangleOBB trailer = RectangleOBB(trailer_length_, width_, 0, trailer_center);

        shapeGroup_->addToGroup(std::make_shared<const RectangleOBB>(head));
        shapeGroup_->addToGroup(std::make_shared<const RectangleOBB>(trailer));
    }

    void Truck::addToGroup(ShapeConstPtr shape) { shapeGroup_->addToGroup(shape); }

    std::vector<ShapeConstPtr> Truck::unpack() const { return shapeGroup_->unpack(); }

#if ENABLE_SERIALIZER
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Truck &);
}

serialize::ICollisionObjectExport *Truck::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

} // namespace collision
