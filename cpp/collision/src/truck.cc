#include "collision/truck.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/utils.h"

namespace collision {

    void Truck::init_truck()
    {
        // Add the two shapes to the truck
        // we gotta count the distance yay?
        // ignore orientation for now
        init();
    }

    void Truck::init()
    {
        double head_length = (2 * length_ - 2 * trailer_length_ - trailer_dist_) / 2;
        Eigen::Vector2d head_center = center_ + Eigen::Vector2d(abs(length_ - head_length), width_ * sin(orientation_));
        RectangleOBB head = RectangleOBB(head_length, width_, orientation_, head_center);

        Eigen::Vector2d trailer_center = center_ - Eigen::Vector2d(abs(length_ - trailer_length_), -sin(orientation_) / width_ );
        RectangleOBB trailer = RectangleOBB(trailer_length_, width_, 0, trailer_center);

        this->addToGroup(std::make_shared<const RectangleOBB>(head));
        this->addToGroup(std::make_shared<const RectangleOBB>(trailer));
    }

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
