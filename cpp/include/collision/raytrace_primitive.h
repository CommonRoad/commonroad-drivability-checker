#ifndef CPP_COLLISION_RAYTRACE_PRIMITIVE_H_
#define CPP_COLLISION_RAYTRACE_PRIMITIVE_H_

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace collision {
namespace raytrace {

using namespace std;

struct Point {
 public:
  Point();
  Point(const Eigen::Vector2d &pnt);
  double x;
  double y;
  Point(const Point &);

  double getCoord(int axis) const {
    if (!axis)
      return x;
    else
      return y;
  }
};

void RaytraceDebugOutput(const char *);
void RaiseRaytraceError(const char *);

bool onSegment(Point p, Point q, Point r);

int orientation(Point p, Point q, Point r);

inline double det(double a, double b, double c, double d);

bool lineLineIntersect(double x1, double y1,           // Line 1 start
                       double x2, double y2,           // Line 1 end
                       double x3, double y3,           // Line 2 start
                       double x4, double y4,           // Line 2 end
                       double &ixOut, double &iyOut);  // Output

int findLineCircleIntersections(double cx, double cy, double radius,
                                const Eigen::Vector2d &p1,
                                const Eigen::Vector2d &p2,
                                std::vector<Eigen::Vector2d> &inters);

bool doIntersect(Point p1, Point q1, Point p2, Point q2,
                 std::vector<Point> &inters);

}  // namespace raytrace
}  // namespace collision

#endif /* CPP_COLLISION_RAYTRACE_PRIMITIVE_H_ */
