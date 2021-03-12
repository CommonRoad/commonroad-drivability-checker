#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <vector>
#include <Eigen/Dense>
#include <optional>

#include "geometry/segment.h"
#include "geometry/curvilinear_coordinate_system.h"

namespace py = pybind11;

#ifdef PY_WRAPPER_MODULE_GEOMETRY
void init_module_geometry(py::module &m);
#endif

PYBIND11_MODULE(pycrccosy, m) {
#ifdef PY_WRAPPER_MODULE_GEOMETRY
  init_module_geometry(m);
#endif
}

void init_module_geometry(py::module &m) {

  py::class_<geometry::Segment>(m, "Segment")
      .def(py::init<const Eigen::Vector2d &, Eigen::Vector2d &, Eigen::Vector2d &, Eigen::Vector2d &>())
      .def("pt_1", &geometry::Segment::pt_1)
      .def("pt_2", &geometry::Segment::pt_2)
      .def("length", &geometry::Segment::length)
      .def("normal_segment_start", &geometry::Segment::normalSegmentStart)
      .def("normal_segment_end", &geometry::Segment::normalSegmentEnd);

  py::class_ < geometry::CurvilinearCoordinateSystem,
      std::shared_ptr < geometry::CurvilinearCoordinateSystem > > (m, "CurvilinearCoordinateSystem")
          .def(py::init<const geometry::EigenPolyline &>())
          .def(py::init<const geometry::EigenPolyline &, const double, const double>())
          .def("length", &geometry::CurvilinearCoordinateSystem::length)
          .def("reference_path", &geometry::CurvilinearCoordinateSystem::referencePath)
          .def("projection_domain", &geometry::CurvilinearCoordinateSystem::projectionDomainBorder)
          .def("curvilinear_projection_domain",
               &geometry::CurvilinearCoordinateSystem::curvilinearProjectionDomainBorder)
          .def("segments_longitudinal_coordinates", &geometry::CurvilinearCoordinateSystem::segmentsLongitudinalCoordinates)
          .def("set_curvature", &geometry::CurvilinearCoordinateSystem::setCurvature)
          .def("curvature_range", &geometry::CurvilinearCoordinateSystem::curvatureRange)
          .def("maximum_curvature_radius", &geometry::CurvilinearCoordinateSystem::maximumCurvatureRadius)
          .def("minimum_curvature_radius", &geometry::CurvilinearCoordinateSystem::minimumCurvatureRadius)
          .def("maximum_curvature", &geometry::CurvilinearCoordinateSystem::maximumCurvature)
          .def("minimum_curvature", &geometry::CurvilinearCoordinateSystem::minimumCurvature)
          .def("normal", &geometry::CurvilinearCoordinateSystem::normal)
          .def("tangent", &geometry::CurvilinearCoordinateSystem::tangent)
          .def("convert_to_cartesian_coords", &geometry::CurvilinearCoordinateSystem::convertToCartesianCoords)
          .def("cartesian_point_inside_projection_domain",
               &geometry::CurvilinearCoordinateSystem::cartesianPointInProjectionDomain)
          .def("determine_subset_of_polygon_within_projection_domain",
               &geometry::CurvilinearCoordinateSystem::determineSubsetOfPolygonWithinProjectionDomain)
          .def("determine_subsets_of_multi_polygons_within_projection_domain", [](
              geometry::CurvilinearCoordinateSystem &cosy,
              const std::vector<geometry::EigenPolyline> &polygons,
              const std::vector<int> groups_of_polygons,
              const int num_omp_threads) {
            std::vector<geometry::EigenPolyline> polygons_in_projection_domain;
            std::vector<int> groups_of_polygons_in_projection_domain;
            cosy.determineSubsetsOfMultiPolygonsWithinProjectionDomain(
                polygons,
                groups_of_polygons,
                num_omp_threads,
                polygons_in_projection_domain,
                groups_of_polygons_in_projection_domain);
            py::tuple return_val(2);
            return_val[0] = py::cast(polygons_in_projection_domain);
            return_val[1] = py::cast(groups_of_polygons_in_projection_domain);
            return return_val;
          })
          .def("determine_subset_of_polygon_within_curvilinear_projection_domain",
               &geometry::CurvilinearCoordinateSystem::determineSubsetOfPolygonWithinCurvilinearProjectionDomain)
          .def("convert_to_curvilinear_coords", [](geometry::CurvilinearCoordinateSystem &cosy, double x, double y) {
            Eigen::Vector2d curvilinear_coord = cosy.convertToCurvilinearCoords(x, y);
            return py::cast(curvilinear_coord);
          })
          .def("convert_to_curvilinear_coords_and_get_segment_idx", [](
              geometry::CurvilinearCoordinateSystem &cosy, double x, double y) {
            int idx = -1;
            Eigen::Vector2d tmp = cosy.convertToCurvilinearCoordsAndGetSegmentIdx(x, y, idx);
            std::vector<double> pos;
            pos.push_back(tmp(0));
            pos.push_back(tmp(1));
            py::list out;
            out.append(py::array(2, pos.data()));
            out.append(idx);
            return out;
          })
          .def("convert_list_of_polygons_to_curvilinear_coords_and_rasterize", [](
              geometry::CurvilinearCoordinateSystem &cosy,
              const std::vector<geometry::EigenPolyline> &polygons,
              const std::vector<int> groups_of_polygons,
              int num_polygon_groups,
              int num_omp_threads) {
            std::vector<std::vector<geometry::EigenPolyline>> transformed_polygon;
            std::vector<std::vector<geometry::EigenPolyline>> transformed_polygon_rasterized;

            cosy.convertListOfPolygonsToCurvilinearCoordsAndRasterize(
                polygons, groups_of_polygons, num_polygon_groups, num_omp_threads,
                transformed_polygon, transformed_polygon_rasterized);

            py::tuple return_val(2);
            return_val[0] = py::cast(transformed_polygon);
            return_val[1] = py::cast(transformed_polygon_rasterized);
            return return_val;
          })
          .def("convert_rectangle_to_cartesian_coords", [](
              geometry::CurvilinearCoordinateSystem &cosy,
              double s_lo, double s_hi, double l_lo, double l_hi
          ) {
            std::vector<geometry::EigenPolyline> triangle_mesh;
            geometry::EigenPolyline transformed_rectangle = cosy.convertRectangleToCartesianCoords(
                s_lo, s_hi, l_lo, l_hi, triangle_mesh);

            py::tuple return_val(2);
            return_val[0] = py::cast(transformed_rectangle);
            return_val[1] = py::cast(triangle_mesh);
            return return_val;
          })
          .def("convert_list_of_points_to_curvilinear_coords",
              &geometry::CurvilinearCoordinateSystem::convertListOfPointsToCurvilinearCoords);
}

