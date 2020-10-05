#include "collision/plugins/triangulation/triangulate.h"
#include <math.h>

#define ENABLE_CGAL 0
#if ENABLE_TRIANGULATION
#if ENABLE_CGAL

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#elif ENABLE_TRIANGLE

#define REAL double
#define VOID int

extern "C" {

#include "triangle.h"
}

#include "string.h"
#endif

namespace collision {
namespace triangulation {

int do_triangulate_aabb(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out) {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::min();
  for (auto vert : vertices) {
    min_x = std::min(min_x, vert(0));
    min_y = std::min(min_y, vert(1));
    max_x = std::max(max_x, vert(0));
    max_y = std::max(max_y, vert(1));
  }

  if (!isinf(min_x) && !isinf(max_x) && !isinf(min_y) && !isinf(max_y)) {
    auto v1 = Eigen::Vector2d(min_x, min_y);
    auto v2 = Eigen::Vector2d(max_x, min_y);
    auto v3 = Eigen::Vector2d(max_x, max_y);
    auto v4 = Eigen::Vector2d(min_x, max_y);
    triangles_out.emplace_back(new collision::Triangle(v1, v2, v3));
    triangles_out.emplace_back(new collision::Triangle(v3, v4, v1));
  }

  return 0;
}

#if ENABLE_CGAL

int do_triangulate(std::vector<Eigen::Vector2d> vertices,
                   std::vector<collision::TriangleConstPtr> &triangles_out) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Exact_predicates_tag Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, Itag>
      CDT;
  typedef CDT::Point Point;
  typedef CGAL::Polygon_2<K> Polygon_2;

  CDT cdt;

  Polygon_2 polygon1;
  for (auto &el : vertices) {
    polygon1.push_back(Point(el[0], el[1]));
  }
  cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(),
                        true);

  for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
       fit != cdt.finite_faces_end(); ++fit) {
    Eigen::Vector2d vert1(cdt.triangle(fit)[0].x(), cdt.triangle(fit)[0].y());
    Eigen::Vector2d vert2(cdt.triangle(fit)[1].x(), cdt.triangle(fit)[1].y());
    Eigen::Vector2d vert3(cdt.triangle(fit)[2].x(), cdt.triangle(fit)[2].y());
    triangles_out.emplace_back(new collision::Triangle(vert1, vert2, vert3));
  }
  return 0;
}

int do_triangulateQuality(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out,
    TriangulationQuality qual) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_vertex_base_2<K> Vb;
  typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
  typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
  typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
  typedef CDT::Vertex_handle Vertex_handle;
  typedef CDT::Point Point;
  typedef CGAL::Polygon_2<K> Polygon_2;

  if (qual.repr_type == MESH_QUALITY_REPRESENTATION_TRIANGLE) {
          double sinangle=sin(qual.quality_triangle/90*M_PI));
          qual.quality_b_cgal = sinangle * sinangle;
  }

  Polygon_2 polygon1;
  for (auto &el : vertices) {
    polygon1.push_back(Point(el[0], el[1]));
  }
  CDT cdt;
  cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(),
                        true);

  CGAL::refine_Delaunay_mesh_2(cdt, Criteria(quality_b, 0));

  for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
       fit != cdt.finite_faces_end(); ++fit) {
    Eigen::Vector2d vert1(cdt.triangle(fit)[0].x(), cdt.triangle(fit)[0].y());
    Eigen::Vector2d vert2(cdt.triangle(fit)[1].x(), cdt.triangle(fit)[1].y());
    Eigen::Vector2d vert3(cdt.triangle(fit)[2].x(), cdt.triangle(fit)[2].y());
    triangles_out.emplace_back(new collision::Triangle(vert1, vert2, vert3));
  }

  return 0;
}

#elif ENABLE_TRIANGLE

#define FREE_IF_NONZERO(x) \
  {                        \
    if (x) free(x);        \
  }

int triangulate_helper(std::vector<Eigen::Vector2d> vertices,
                       std::vector<collision::TriangleConstPtr> &triangles_out,
                       char *mode) {
  struct triangulateio in, mid, out, vorout;

  /* Make necessary initializations so that Triangle can return a */
  /*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

  memset(&in, 0, sizeof(triangulateio));
  memset(&mid, 0, sizeof(triangulateio));
  memset(&out, 0, sizeof(triangulateio));
  memset(&vorout, 0, sizeof(triangulateio));

  in.numberofpoints = vertices.size();
  in.numberofpointattributes = 0;
  in.pointlist = (REAL *)malloc(in.numberofpoints * 2 * sizeof(REAL));
  for (int i = 0; i < in.numberofpoints; i++) {
    in.pointlist[i * 2] = vertices[i][0];
    in.pointlist[i * 2 + 1] = vertices[i][1];
  }

  in.pointattributelist = 0;

  in.pointmarkerlist = 0;

  in.numberofsegments = in.numberofpoints;
  in.segmentlist = (int *)malloc(in.numberofsegments * 2 * sizeof(int));
  for (int cc1 = 0; cc1 < in.numberofpoints - 1; cc1++) {
    in.segmentlist[cc1 * 2] = cc1;
    in.segmentlist[cc1 * 2 + 1] = cc1 + 1;
  }
  in.segmentlist[(in.numberofpoints - 1) * 2] = in.numberofpoints - 1;
  in.segmentlist[(in.numberofpoints - 1) * 2 + 1] = 0;

  in.numberofholes = 0;
  in.numberofregions = 0;

  /* Triangulate the points.  Switches are chosen to read and write a  */
  /*   PSLG (p), preserve the convex hull (c), number everything from  */
  /*   zero (z), assign a regional attribute to each element (A), and  */
  /*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
  /*   neighbor list (n).                                              */

  triangulate(mode, &in, &mid, &vorout);

  for (int cc1 = 0; cc1 < mid.numberoftriangles; cc1++) {
    double x_1 = mid.pointlist[mid.trianglelist[cc1 * 3] * 2];
    double y_1 = mid.pointlist[mid.trianglelist[cc1 * 3] * 2 + 1];
    double x_2 = mid.pointlist[mid.trianglelist[cc1 * 3 + 1] * 2];
    double y_2 = mid.pointlist[mid.trianglelist[cc1 * 3 + 1] * 2 + 1];
    double x_3 = mid.pointlist[mid.trianglelist[cc1 * 3 + 2] * 2];
    double y_3 = mid.pointlist[mid.trianglelist[cc1 * 3 + 2] * 2 + 1];
    triangles_out.push_back(std::make_shared<const collision::Triangle>(
        Eigen::Vector2d(x_1, y_1), Eigen::Vector2d(x_2, y_2),
        Eigen::Vector2d(x_3, y_3)));
  }

  FREE_IF_NONZERO(in.pointlist);
  FREE_IF_NONZERO(in.segmentlist);
  FREE_IF_NONZERO(mid.pointlist);
  FREE_IF_NONZERO(mid.pointattributelist);
  FREE_IF_NONZERO(mid.pointmarkerlist);
  FREE_IF_NONZERO(mid.trianglelist);
  FREE_IF_NONZERO(mid.triangleattributelist);
  FREE_IF_NONZERO(mid.trianglearealist);
  FREE_IF_NONZERO(mid.neighborlist);
  FREE_IF_NONZERO(mid.segmentlist);
  FREE_IF_NONZERO(mid.segmentmarkerlist);
  FREE_IF_NONZERO(mid.edgelist);
  FREE_IF_NONZERO(mid.edgemarkerlist);
  FREE_IF_NONZERO(vorout.pointlist);
  FREE_IF_NONZERO(vorout.pointattributelist);
  FREE_IF_NONZERO(vorout.edgelist);
  FREE_IF_NONZERO(vorout.normlist);
  FREE_IF_NONZERO(out.pointlist);
  FREE_IF_NONZERO(out.pointattributelist);
  FREE_IF_NONZERO(out.trianglelist);
  FREE_IF_NONZERO(out.triangleattributelist);

  return 0;
}

int do_triangulate(std::vector<Eigen::Vector2d> vertices,
                   std::vector<collision::TriangleConstPtr> &triangles_out) {
  triangulate_helper(vertices, triangles_out, (char *)("pzQ"));
  return 0;
}

int do_triangulateQuality(
    std::vector<Eigen::Vector2d> vertices,
    std::vector<collision::TriangleConstPtr> &triangles_out,
    TriangulationQuality qual) {
  // preconditions: qual.quality_b_cgal is positive
  if (qual.use_quality) {
    if (qual.repr_type == triangulation::MESH_QUALITY_REPRESENTATION_CGAL) {
      qual.quality_triangle =
          asin(sqrt(abs(qual.quality_b_cgal))) / M_PI * 90;  // abs is redundant
    }
    char buffer[64];
    int ret =
        snprintf(buffer, sizeof buffer, "pzQq%.0f", qual.quality_triangle);
    buffer[63] = 0;
    triangulate_helper(vertices, triangles_out, buffer);
  } else {
    do_triangulate(vertices, triangles_out);
  }
  return 0;
}

#endif

}  // namespace triangulation
}  // namespace collision
#endif
