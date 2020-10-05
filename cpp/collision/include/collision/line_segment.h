#ifndef CPP_COLLISION_LINE_SEGMENT_H_
#define CPP_COLLISION_LINE_SEGMENT_H_

#include <Eigen/Dense>
#include <vector>
#include "collision/raytrace_primitive.h"

namespace collision

{

class LineSegment {
 public:
  LineSegment(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2) {
    point1_.x = point1[0];
    point1_.y = point1[1];
    point2_.x = point2[0];
    point2_.y = point2[1];
  }
  LineSegment(raytrace::Point point1, raytrace::Point point2) {
    point1_ = point1;
    point2_ = point2;
  }
  LineSegment(const LineSegment &ls2) {
    point1_ = ls2.point1_;
    point2_ = ls2.point2_;
  }
  raytrace::Point point1() const { return point1_; }

  raytrace::Point point2() const { return point2_; }

  void set_point_1(const raytrace::Point &point1) { point1_ = point1; }

  void set_point_2(const raytrace::Point &point2) { point2_ = point2; }

  void swap(void) {
    raytrace::Point point3 = point2_;
    point2_ = point1_;
    point1_ = point3;
  }

  bool intersect(LineSegment segment2,
                 std::vector<Eigen::Vector2d> &inters) const {
    std::vector<raytrace::Point> inters1;
    bool res =
        raytrace::doIntersect(this->point1_, this->point2_, segment2.point1_,
                              segment2.point2_, inters1);
    for (auto &el : inters1) {
      Eigen::Vector2d v2(el.x, el.y);
      inters.push_back(v2);
    }

    return res;
  }

 private:
  raytrace::Point point1_;
  raytrace::Point point2_;
};

}  // namespace collision

#endif /* CPP_COLLISION_LINE_SEGMENT_H_ */
