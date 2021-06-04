#include "collision/shape_group.h"
#include <Eigen/Dense>
#include <vector>

namespace collision {

class Truck : public ShapeGroup {
public:
    Truck(const Eigen::Vector2d &_center, const std::vector<double> params);
    virtual ~Truck() {}

private:
    Eigen::Vector2d center_;
    double length_;
    double width_;
    double trailer_dist_;
    double trailer_length_;
    double orientation_;
    void init();
    void init_simple();
    void init_rotated();

}

}
