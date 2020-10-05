#include "collision/application.h"

#include <math.h>
#include <Eigen/Dense>

#include "collision/line_segment.h"
#include "collision/narrowphase/point.h"
#include "collision/raytrace_primitive.h"

#include "collision/raytrace_utils.h"

namespace collision {
namespace raytrace {

struct Vector2dSort_X {
  bool operator()(const Eigen::Vector2d &i, const Eigen::Vector2d &j) {
    return (i[0] <= j[0]);
  }
} vector2dSortX;

struct Vector2dSort_Y {
  bool operator()(const Eigen::Vector2d &i, const Eigen::Vector2d &j) {
    return (i[1] <= j[1]);
  }
} vector2dSortY;

struct StartsLineSegmentSort_X {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point1().x <= j.point1().x);
  }
} startsSortX;

struct StartsLineSegmentSort_X_goe {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point1().x >= j.point1().x);
  }
} startsSortXgoe;

struct StartsLineSegmentSort_Y {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point1().y <= j.point1().y);
  }
} startsSortY;

struct StartsLineSegmentSort_Y_goe {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point1().y >= j.point1().y);
  }
} startsSortY_goe;

struct EndsLineSegmentSort_X {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point2().x <= j.point2().x);
  }
} endsSortX;

struct EndsLineSegmentSort_X_goe {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point2().x >= j.point2().x);
  }
} endsSortX_goe;

struct EndsLineSegmentSort_Y {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point2().y <= j.point2().y);
  }
} endsSortY;

struct EndsLineSegmentSort_Y_goe {
  bool operator()(LineSegment i, LineSegment j) {
    return (i.point2().y >= j.point2().y);
  }
} endsSortY_goe;

bool rayTracePrimitive(
    std::vector<collision::CollisionObjectConstPtr> collision_objects,
    const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
    std::vector<LineSegment> &intersect, bool remove_overlaps) {
  bool res = false;
  std::vector<LineSegment> res2;  // intersecting segments storage
  int counter = 0;
  for (auto &obj : collision_objects) {
    res = obj->rayTrace(point1, point2, res2) || res;
    counter++;
  }

#if RAYTRACE_DEBUG
  std::stringstream out;
  out << "[Raytrace] Obstacle CollisionObjects count: " << counter << std::endl;
  out << "[Raytrace] Returned segments: " << res2.size() << std::endl;
  raytrace::RaytraceDebugOutput(out.str().c_str());
#endif

  if (remove_overlaps) {
    int axis =
        (fabs(point1[0] - point2[0]) <=
         fabs(point1[1] - point2[1]));  // 0 - greater in X axis projection
    collision::raytrace::rayTraceRemoveOverlaps(res2, intersect, axis);
  } else {
    intersect.insert(intersect.end(), res2.begin(), res2.end());
  }
  return res;
}

bool rayTracePostprocess(const Eigen::Vector2d &point1,
                         const Eigen::Vector2d &point2,
                         std::vector<Eigen::Vector2d> inters1,
                         std::vector<LineSegment> &intersect,
                         const collision::CollisionObject *obj) {
  double eps = 0.0000001;

#if RAYTRACE_DEBUG
  std::stringstream out;
  out << "[Raytrace] Postprocessing points: " << inters1.size() << std::endl;
  raytrace::RaytraceDebugOutput(out.str().c_str());
#endif

  if (inters1.size() == 2) {
    intersect.push_back(LineSegment(inters1[0], inters1[1]));
    return true;
  } else if (inters1.size() == 1) {
    if (obj->collide(collision::Point(point2), CollisionRequest(COL_DEFAULT))) {
      intersect.push_back(LineSegment(point2, inters1[0]));
      return true;
    } else {
      if (obj->collide(collision::Point(point1),
                       CollisionRequest(COL_DEFAULT)))  // optional
      {
        intersect.push_back(LineSegment(point1, inters1[0]));
        return true;
      }
    }
  } else if (inters1.size() == 0) {
    // second check optional
    if (obj->collide(collision::Point(point2), CollisionRequest(COL_DEFAULT)) &&
        obj->collide(collision::Point(point1), CollisionRequest(COL_DEFAULT))) {
      intersect.push_back(LineSegment(point1, point2));
      return true;
    }
  } else {
    // more than two points returned - must filter duplicates

    // std::vector<Eigen::Vector2d> inters1_copy;
    // inters1_copy=inters1;

    // std::sort(inters1_copy.begin(),inters1_copy.end(), vector2dSortX);

    int numFound = 0;
    std::vector<Eigen::Vector2d> inters2;
    // raytrace::RaiseRaytraceError();

#if RAYTRACE_DEBUG
    std::ostringstream debugO;
    debugO << "[RayTrace] [Raytrace Point]" << inters1[0][0] << " ";
    debugO << inters1[0][1] << " ;";
#endif
    unsigned int cc1 = 1;
    for (; cc1 < inters1.size(); cc1++) {
      if ((pow(inters1[cc1][0] - inters1[0][0], 2) +
           pow(inters1[cc1][1] - inters1[0][1], 2)) >= eps) {
        if (numFound < 1) {
          numFound++;
          inters2.push_back(inters1[cc1]);
#if RAYTRACE_DEBUG
          debugO << "[RayTrace] [Raytrace Point]" << inters1[cc1][0] << " ";
          debugO << inters1[cc1][1] << " ;";
#endif

        } else {
          // raytrace::RaiseRaytraceError("more than two points found after
          // LineSegment intersect postprocessing\n");

          // obj->to_string(debugO);
          // debugO << "[RayTrace] [Raytrace Point]" <<  inters1[cc1][0] << " ";
          // debugO << inters1[cc1][1] << " ;";

          break;
        }
      }
    }
#if RAYTRACE_DEBUG
    raytrace::RaytraceDebugOutput(debugO.str().c_str());
#endif

    if (numFound >= 1) {
      intersect.push_back(LineSegment(inters1[0], inters2[0]));
    } else {
      inters2.push_back(inters1[0]);  // only one point in intersection
      return rayTracePostprocess(point1, point2, inters2, intersect, obj);
    }

    return true;
  }
  return true;
}

int rayTraceRemoveOverlaps(std::vector<LineSegment> intersect,
                           std::vector<LineSegment> &out_vec, int axis) {
  std::vector<LineSegment> starts = intersect;
  std::vector<LineSegment> ends = intersect;
  std::vector<bool> remove_its(intersect.size());
  // todo zeromemory?

  for (auto &it : starts) {
    if (it.point1().getCoord(axis) > it.point2().getCoord(axis)) {
      it.swap();
    }
  }
  if (!axis) {
    std::sort(starts.begin(), starts.end(), startsSortX);
  } else {
    std::sort(starts.begin(), starts.end(), startsSortY);
  }
  // std::sort(ends.begin(), ends.end(), endsSortX);

  std::vector<LineSegment>::iterator starts_it = starts.begin();
  while (starts_it < starts.end()) {
    bool repeat = 1;
    auto starts_it2 = starts_it + 1;

    while (repeat) {
      std::vector<LineSegment>::iterator maxend = starts_it;
      while ((starts_it2 < starts.end()) &&
             (starts_it2)->point1().getCoord(axis) <=
                 (starts_it)->point2().getCoord(axis)) {
        remove_its[starts_it2 - starts.begin()] = true;
        if ((starts_it2)->point2().getCoord(axis) >=
            maxend->point2().getCoord(axis)) {
          maxend = starts_it2;
        }
        starts_it2++;
      }
      if (maxend != starts_it) {
        starts_it->set_point_2(maxend->point2());
      } else {
        starts_it = starts_it2;
        repeat = 0;
      }
    }
  }

  for (unsigned int cc1 = 0; cc1 < starts.size(); cc1++) {
    if (!remove_its[cc1]) {
      out_vec.push_back(starts[cc1]);
    }
  }

  return 0;
}

}  // namespace raytrace
}  // namespace collision
