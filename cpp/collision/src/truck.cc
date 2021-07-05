#include "collision/truck.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/utils.h"

namespace collision {

    void Truck::init_truck()
    {
        // Add the two shapes to the truck
        // we gotta count the distance yay?
        // ignore orientation for now
        if (orientation_ == 0) {
            init_simple();
            return;
        }
        init_rotated();
    }

    void Truck::init_simple()
    {
        double head_length = (2 * length_ - 2 * trailer_length_ - trailer_dist_) / 2;
        Eigen::Vector2d head_center = center_ - Eigen::Vector2d(head_length - length_, 0);
        RectangleAABB head = RectangleAABB(head_length, width_, head_center);

        Eigen::Vector2d trailer_center = center_ - Eigen::Vector2d(length_ - trailer_length_, 0);
        RectangleAABB trailer = RectangleAABB(trailer_length_, width_, trailer_center);

        this->addToGroup(std::make_shared<const RectangleAABB>(head));
        this->addToGroup(std::make_shared<const RectangleAABB>(trailer));
    }

    void Truck::init_rotated()
    {}

    std::vector<ShapeConstPtr> Truck::unpack() const { return ShapeGroup::unpack(); }

#if ENABLE_SERIALIZER
namespace serialize {
ICollisionObjectExport *exportObject(const collision::Truck &);
}

serialize::ICollisionObjectExport *Truck::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

} // namespace collision
