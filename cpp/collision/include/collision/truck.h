#ifndef TRUCK_H_
#define TRUCK_H_

#include "collision/shape_group.h"
#include "collision/collision_object.h"
#include <Eigen/Dense>
#include <vector>

namespace collision {

typedef std::shared_ptr<Truck> TruckPtr;
typedef std::shared_ptr<const Truck> TruckConstPtr;

class Truck : public CollisionObjectEx,
              public ICollisionContainer,
              public IFCLCollisionObjectGroup {

public:
    Truck(const Eigen::Vector2d &center=Eigen::Vector2d(0, 0),
          const std::map<std::string, double> params=std::map<std::string, double>())
    {
        center_ = center;
        length_ = params.count("length") ? params.at("length") : 7;
        width_ = params.count("width") ? params.at("width") : 2;
        trailer_length_ = params.count("trailer_length") ? params.at("trailer_length") : 5;
        orientation_ = params.count("orientation") ? params.at("orientation") : 0;
        hitch_angle_ = params.count("hitch") ? params.at("hitch") : 0;
        shapeGroup_ = new ShapeGroup();
        init();
    }
    virtual ~Truck() {}
    CollisionObjectConstPtr timeSlice(
            int time_idx, CollisionObjectConstPtr shared_ptr_this) const override;
    int queryContainedObjectIndexList(const CollisionObject *,
                                      std::list<int> &retlist) const override;
    int getCollisionObjects(std::vector<fcl::CollisionObject<FCL_PRECISION> *>
                            &obj_vec) const override;
    void addToGroup(ShapeConstPtr shape);
    int elementCount(void) const;
    std::vector<ShapeConstPtr> unpack() const;

#if ENABLE_SERIALIZER
    serialize::ICollisionObjectExport *exportThis(void) const;
#endif

private:
    Eigen::Vector2d center_;
    double length_;
    double width_;
    double trailer_length_;
    double orientation_;
    double hitch_angle_;
    ShapeGroup* shapeGroup_;

    Truck(const Truck &);
    Truck &operator=(const Truck &) { return *this; };
    void init();
    friend std::vector<ShapeConstPtr> ShapeGroup::getShapes();
    friend std::unordered_map<const CollisionObject *, std::list<int>> ShapeGroup::getShapesMap();
};

} // namespace collision

#endif
