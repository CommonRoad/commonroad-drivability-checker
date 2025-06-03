#include "collision/application.h"

#include <cmath>
#include <iostream>
#include <vector>

#include "collision/raytrace_primitive.h"

namespace collision {
namespace raytrace {

Point::Point() {
  x = 0;
  y = 0;
};
Point::Point(const Eigen::Vector2d &pnt) : x(pnt.x()), y(pnt.y()){};
Point::Point(const Point &pnt) : x(pnt.x), y(pnt.y){};

// Adapted from
// http://csharphelper.com/blog/2014/09/determine-where-a-line-intersects-a-circle-in-c/
int findLineCircleIntersections(double cx, double cy, double radius,
                                const Eigen::Vector2d &p1,
                                const Eigen::Vector2d &p2,
                                std::vector<Eigen::Vector2d> &inters) {
  double dx, dy, A, B, C, det, t;

  dx = p2.x() - p1.x();
  dy = p2.y() - p1.y();

  A = dx * dx + dy * dy;
  B = 2 * (dx * (p1.x() - cx) + dy * (p1.y() - cy));
  C = (p1.x() - cx) * (p1.x() - cx) + (p1.y() - cy) * (p1.y() - cy) -
      radius * radius;

  det = B * B - 4 * A * C;
  if ((A <= 0.0000001) || (det < 0)) {
    // No real solutions.
    return 0;
  } else if (det == 0) {
    // One solution.
    t = -B / (2 * A);
    inters.push_back(Eigen::Vector2d(p1.x() + t * dx, p1.y() + t * dy));
    return 1;
  } else {
    // Two solutions.
    t = (double)((-B + sqrt(det)) / (2 * A));
    inters.push_back(Eigen::Vector2d(p1.x() + t * dx, p1.y() + t * dy));

    t = (double)((-B - sqrt(det)) / (2 * A));
    inters.push_back(Eigen::Vector2d(p1.x() + t * dx, p1.y() + t * dy));
    return 2;
  }
}

void RaytraceDebugOutput(const char *message) {
#if RAYTRACE_DEBUG
  std::cout << message;
#endif
}

void RaiseRaytraceError(const char *message) {
#if RAYTRACE_DEBUG
  std::cout << message;
#endif
}

// Adapted from
// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
bool onSegment(Point p, Point q, Point r) {
  if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) &&
      q.y >= min(p.y, r.y))
    return true;

  return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r) {
  // See 10th slides from following link for derivation of the formula
  // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (val == 0) return 0;  // colinear

  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

inline double det(double a, double b, double c, double d) {
  return a * d - b * c;
}

bool lineLineIntersect(double x1, double y1,          // Line 1 start
                       double x2, double y2,          // Line 1 end
                       double x3, double y3,          // Line 2 start
                       double x4, double y4,          // Line 2 end
                       double &ixOut, double &iyOut)  // Output
{
  // http://mathworld.wolfram.com/Line-LineIntersection.html

  double detL1 = det(x1, y1, x2, y2);
  double detL2 = det(x3, y3, x4, y4);
  double x1mx2 = x1 - x2;
  double x3mx4 = x3 - x4;
  double y1my2 = y1 - y2;
  double y3my4 = y3 - y4;

  double xnom = det(detL1, x1mx2, detL2, x3mx4);
  double ynom = det(detL1, y1my2, detL2, y3my4);
  double denom = det(x1mx2, y1my2, x3mx4, y3my4);
  if (denom == 0.0)  // Lines don't seem to cross
  {
    ixOut = NAN;
    iyOut = NAN;
    return false;
  }

  ixOut = xnom / denom;
  iyOut = ynom / denom;
  if (!isfinite(ixOut) || !isfinite(iyOut))  // Probably a numerical issue
    return false;

  return true;  // All OK
}

// Part taken from
// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ The
// main function that returns true if line segment 'p1q1' and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2,
                 std::vector<Point> &inters) {
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4) {
    double int_x = 0;
    double int_y = 0;
    if (lineLineIntersect(p1.x, p1.y, q1.x, q1.y, p2.x, p2.y, q2.x, q2.y, int_x,
                          int_y)) {
      Point intersection1;
      intersection1.x = int_x;
      intersection1.y = int_y;
      inters.push_back(intersection1);
      return true;
    } else {
      raytrace::RaiseRaytraceError("numeric issue with points intersection");
      return false;
    }
  }

  // if p1, p2, q1, q2 are collinear
  if (o1 == 0 && o2 == 0) {
    int outnum = 0;

    // if p2 or q2 lies on p1q1

    if (onSegment(p1, p2, q1)) {
      outnum++;
      inters.push_back(p2);
    }
    if (outnum == 2) return true;

    if (onSegment(p1, q2, q1)) {
      outnum++;
      inters.push_back(q2);
    }

    if (outnum == 2) return true;

    // if p1 or q1 lies on p2q2

    if (onSegment(p2, p1, q2)) {
      outnum++;
      inters.push_back(p1);
    }

    if (outnum == 2) return true;

    if (onSegment(p2, q1, q2)) {
      outnum++;
      inters.push_back(q1);
    }

    // if(outnum==2) return true;
    return true;
  }

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1)) {
    inters.push_back(p2);
    return true;
  }

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1)) {
    inters.push_back(q2);
    return true;
  }

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2)) {
    inters.push_back(p1);
    return true;
  }

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2)) {
    inters.push_back(q1);
    return true;
  }

  return false;  // Doesn't fall in any of the above cases
}
}  // namespace raytrace
}  // namespace collision
