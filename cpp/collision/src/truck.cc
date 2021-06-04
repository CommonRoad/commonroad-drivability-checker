#include "collision/truck.h"

namespace collision {

    Truck(const Eigen::Vector2d &_center, const std::vector<double> params)
    {
        center_ = _center;
        length_ = params[0];
        width_ = params[1];
        trailer_dist_ = params[2];
        trailer_length_ = params[3];
        orientation_ = params[4];
        init();
    }

    void Truck::init()
    {
        // Add the two shapes to the truck
        // we gotta count the distance yay?
        // ignore orientation for now
        if (orientation == 0) {
            init_simple();
            return;
        }
        init_rotated();
    }

    void Truck::init_simple()
    {
        head_length = (2 * length_ - trailer_length - trailer_dist_) / 2;
        head_center = center_ - Eigen::Vector2d(head_length - length_, 0);
        RectangleAABB head = RectangleAABB(head_length, width_, head_center);

        trailer_center = center_ - Eigen::Vector2d(length_ - trailer_length_);
        RectangleAABB trailer = RectangleAABB(trailer_length_, width_, trailer_center);

        this->addToGroup(head);
        this->addToGroup(trailer);
    }

    void Truck::init_rotated()
    {}
}

