#pragma once

#include "collision/application_settings.h"

#if ENABLE_TRIANGULATION
#include "collision/narrowphase/triangle.h"

namespace collision {
namespace triangulation {

enum TRIANGULATION_METHOD
{
	TRIANGULATION_GPC=0,
	TRIANGULATION_TRIANGLE,
	TRIANGULATION_CGAL
};

class TriangulationQuality {
 public:
  TriangulationQuality(void) {
    use_quality = true;

    mesh_quality = 20;
    bb_only = false;
  }

  TriangulationQuality(double mesh_quality) {
    use_quality = true;

    this->mesh_quality = mesh_quality;
    bb_only = false;
  }

  bool bb_only;
  bool use_quality;
  double mesh_quality;
};

int do_triangulate_aabb(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out);

int do_triangulate(std::vector<Eigen::Vector2d> vertices,
                   std::vector<collision::TriangleConstPtr> &triangles_out, int method);
int do_triangulateQuality(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out, int method,
    TriangulationQuality qual);
}  // namespace triangulation
}  // namespace collision
#endif
