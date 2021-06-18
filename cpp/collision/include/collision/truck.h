#ifndef TRUCK_H_
#define TRUCK_H_

#include "collision/shape_group.h"
#include <Eigen/Dense>
#include <vector>

namespace collision {

class Truck : public ShapeGroup {
public:
    Truck(const Eigen::Vector2d &center, const std::vector<double> params)
    : ShapeGroup()
    {
        center_ = center;
        length_ = params[0];
        width_ = params[1];
        trailer_dist_ = params[2];
        trailer_length_ = params[3];
        orientation_ = params[4];
        hitch_angle_ = params[5];
        init_truck();
    }
    virtual ~Truck() {}

private:
    Eigen::Vector2d center_;
    double length_;
    double width_;
    double trailer_dist_;
    double trailer_length_;
    double orientation_;
    double hitch_angle_;
    void init_truck();
    void init_simple();
    void init_rotated();

};

} // namespace collision

#endif
