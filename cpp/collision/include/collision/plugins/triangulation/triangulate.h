#pragma once

#include "collision/application_settings.h"

#if ENABLE_TRIANGULATION
#include "collision/narrowphase/triangle.h"

namespace collision {
namespace triangulation {

enum MESH_QUALITY_REPRESENTATION {
  MESH_QUALITY_REPRESENTATION_TRIANGLE = 0,
  MESH_QUALITY_REPRESENTATION_CGAL = 1
};

class TriangulationQuality {
 public:
  TriangulationQuality(void) {
    use_quality = true;
    repr_type = MESH_QUALITY_REPRESENTATION_TRIANGLE;
    quality_b_cgal = 0.125;
    quality_triangle = 20;
    bb_only = false;
  }

  TriangulationQuality(double cgal_quality_b, double triangle_quality) {
    use_quality = true;
    repr_type = MESH_QUALITY_REPRESENTATION_TRIANGLE;
    this->quality_b_cgal = cgal_quality_b;
    quality_triangle = triangle_quality;
    bb_only = false;
  }

  bool bb_only;
  bool use_quality;
  MESH_QUALITY_REPRESENTATION repr_type;
  double quality_b_cgal;
  double quality_triangle;
  // double quality_b = 0.125
};

int do_triangulate_aabb(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out);

int do_triangulate(std::vector<Eigen::Vector2d> vertices,
                   std::vector<collision::TriangleConstPtr> &triangles_out);
int do_triangulateQuality(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out,
    TriangulationQuality qual);
}  // namespace triangulation
}  // namespace collision
#endif
