#include "collision/solvers/sat2d/sat2d_checks.h"

namespace collision {
namespace detail {
namespace sat2dChecks {
inline bool overlaps1Way(const Triangle_SAT2D& tri, const OBB_SAT2D& other) {
  for (int c1 = 0; c1 < 3; c1++) {
    double t = other.corner.col(0).dot(tri.axes.col(c1));

    double min_t = t;
    double max_t = t;

    for (int c2 = 1; c2 < 4; ++c2) {
      t = other.corner.col(c2).dot(tri.axes.col(c1));

      if (t < min_t) {
        min_t = t;
      } else if (t > max_t) {
        max_t = t;
      }
    }

    if ((min_t > 1 + tri.origin[c1]) || (max_t < tri.origin[c1])) {
      return false;
    }
  }

  return true;
}

inline bool overlaps1Way(const OBB_SAT2D& obb, const Triangle_SAT2D& other) {
  for (int c1 = 0; c1 < 2; c1++) {
    double t = other.corner.col(0).dot(obb.axes.col(c1));

    double min_t = t;
    double max_t = t;

    for (int c2 = 1; c2 < 3; ++c2) {
      t = other.corner.col(c2).dot(obb.axes.col(c1));

      if (t < min_t) {
        min_t = t;
      } else if (t > max_t) {
        max_t = t;
      }
    }

    if ((min_t > 1 + obb.origin[c1]) || (max_t < obb.origin[c1])) {
      return false;
    }
  }

  return true;
}

bool overlaps(const Triangle_SAT2D& tri, const OBB_SAT2D& obb) {
  return tri.bValid && overlaps1Way(obb, tri) && overlaps1Way(tri, obb);
}

}  // namespace sat2dChecks
int Triangle_SAT2D::opposite_vert[3] = {2, 1, 0};

int Triangle_SAT2D::origin_vert[3] = {0, 0, 1};

}  // namespace detail
}  // namespace collision
