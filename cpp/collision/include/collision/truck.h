#ifndef TRUCK_H_
#define TRUCK_H_

#include "collision/shape_group.h"
#include <Eigen/Dense>
#include <vector>

namespace collision {

typedef std::shared_ptr<Truck> TruckPtr;
typedef std::shared_ptr<const Truck> TruckConstPtr;

class Truck : public ShapeGroup {
public:
    Truck(const Eigen::Vector2d &center=Eigen::Vector2d(0, 0), const std::map<std::string, double> params=std::map<std::string, double>())
    : ShapeGroup()
    {
        center_ = center;
        length_ = params.count("length") ? params.at("length") : 7;
        width_ = params.count("width") ? params.at("width") : 2;
        trailer_dist_ = params.count("trailer_dist") ? params.at("trailer_dist") : 0.5;
        trailer_length_ = params.count("trailer_length") ? params.at("trailer_length") : 5;
        orientation_ = params.count("orientation") ? params.at("orientation") : 0;
        hitch_angle_ = params.count("hitch") ? params.at("hitch") : 0;
        init_truck();
    }
    virtual ~Truck() {}
    std::vector<ShapeConstPtr> unpack() const;

#if ENABLE_SERIALIZER
    serialize::ICollisionObjectExport *exportThis(void) const;
#endif

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

    Truck(const Truck &);
    Truck &operator=(const Truck &) { return *this; };
};

} // namespace collision

#endif
