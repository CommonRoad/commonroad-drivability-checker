#include <Eigen/Dense>
#include <vector>

#include "collision/collision_checker.h"
#include "collision/narrowphase/point.h"
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/sphere.h"
#include "collision/narrowphase/triangle.h"
#include "collision/shape_group.h"

#include "collision/time_variant_collision_object.h"

#include "collision/solvers/boost/boost_collision_queries.h"
#include "collision/solvers/boost/boost_geometry_queries.h"
#include "collision/solvers/boost/boost_helpers.h"

#include "collision/solvers/collision_queries.h"
#include "collision/solvers/distance_queries.h"
#include "collision/solvers/geometry_queries.h"
#include "collision/solvers/trajectory_queries.h"

#include "collision/solvers/accelerators/declarations.h"

#include <iostream>
#include <sstream>
#include <string>

#include <stdexcept>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
//#include <pybind11/embed.h>


#include <Eigen/Dense>

namespace py = pybind11;
template <typename T>
using aligned_vector = collision::aligned_vector<T>;

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

#include "collision/solvers/sat2d/sat2d_checks.h"

typedef Eigen::MatrixXd Matrixd;

typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;

typedef Matrixd::Scalar Scalard;

typedef VectorXi::Scalar Scalari;

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

#include "submodule_util.h"

#ifdef PY_WRAPPER_MODULE_COLLISION
void init_module_collision(py::module &m);
#endif

PYBIND11_MODULE(pycrcc, m) {
#ifdef PY_WRAPPER_MODULE_COLLISION
  init_module_collision(m);
  py::module mutil = m.def_submodule(
      "Util", "Util is a submodule of pycrcc containing auxiliary functions");

  py::module mutil_deprecated = mutil.def_submodule(
      "deprecated", "Deprecated utility functions submodule");
  py::module mutil_experimental = mutil.def_submodule(
      "experimental", "Experimental utility functions submodule");

#if TIME_PROFILE_ENABLED
  mutil.def("perfReport", []() {
    test::report_perfomance();
    return 0;
  });
  mutil.def("perfStart", []() {
    std::cout << "perf started";
    test::init_perfomance_timers();
    return 0;
  });
#endif

  // Please call the functions through the Python wrappers in boundary.py and
  // trajectory_queries.py

  mutil.def(
      "trajectories_collision_static_obstacles_fcl",
      [](py::list &trajectories, collision::ShapeGroup &static_obstacles,
         bool enable_verification, int broadphase_type, int num_cells,
         int num_buckets) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_FCL;
        req.enable_verification = enable_verification;
        req.num_cells = num_cells;
        req.fcl_broadphase_type = broadphase_type;
        req.fcl_num_buckets = num_buckets;
        collision::TrajectoryQueryResultCollisionTime res(&result);

        collision::trajectory_queries::
            trajectories_collision_time_static_obstacles(
                traj_in, static_obstacles, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "trajectories_collision_static_obstacles_grid",
      [](py::list &trajectories, collision::ShapeGroup &static_obstacles,
         bool enable_verification, bool auto_orientation, int num_cells,
         bool optimize_triangles) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_UNIFORMGRID;
        req.enable_verification = enable_verification;
        req.num_cells = num_cells;
        req.auto_orientation = auto_orientation;
        req.optimize_triangles = optimize_triangles;
        collision::TrajectoryQueryResultCollisionTime res(&result);

        int err = collision::trajectory_queries::
            trajectories_collision_time_static_obstacles(
                traj_in, static_obstacles, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "trajectories_collision_static_obstacles_box2d",
      [](py::list &trajectories, collision::ShapeGroup &static_obstacles,
         bool enable_verification) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);
        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_BOX2D;
        req.enable_verification = enable_verification;
        collision::TrajectoryQueryResultCollisionTime res(&result);
        collision::trajectory_queries::
            trajectories_collision_time_static_obstacles(
                traj_in, static_obstacles, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });
  mutil.def(
      "trajectories_enclosure_polygons_static_grid",
      [](py::list &trajectories,
         collision::ShapeGroup &static_obstacles_polygons,
         bool enable_verification, int num_cells) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_UNIFORMGRID;
        req.enable_verification = enable_verification;
        req.num_cells = num_cells;
        collision::TrajectoryQueryResultCollisionTime res(&result);

        collision::trajectory_queries::
            trajectories_enclosure_time_polygons_static(
                traj_in, static_obstacles_polygons, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "trajectories_collision_dynamic_obstacles_grid",
      [](py::list &trajectories, py::list &dynamic_obstacles,
         bool enable_verification, int num_cells) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        aligned_vector<const collision::TimeVariantCollisionObject *>
            obsts_scenario;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();
        if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_UNIFORMGRID;
        req.enable_verification = enable_verification;
        req.num_cells = num_cells;
        collision::TrajectoryQueryResultCollisionTime res(&result);

        int err = collision::trajectory_queries::
            trajectories_collision_time_dynamic_obstacles(
                traj_in, obsts_scenario, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "trajectories_collision_dynamic_obstacles_box2d",
      [](py::list &trajectories, py::list &dynamic_obstacles,
         bool enable_verification) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        aligned_vector<const collision::TimeVariantCollisionObject *>
            obsts_scenario;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();
        if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_BOX2D;
        req.enable_verification = enable_verification;
        collision::TrajectoryQueryResultCollisionTime res(&result);

        int err = collision::trajectory_queries::
            trajectories_collision_time_dynamic_obstacles(
                traj_in, obsts_scenario, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "trajectories_collision_dynamic_obstacles_fcl",
      [](py::list &trajectories, py::list &dynamic_obstacles,
         bool enable_verification, int broadphase_type, int num_cells,
         int num_buckets) {
#if TIME_PROFILE_ENABLED
        STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        aligned_vector<const collision::TimeVariantCollisionObject *>
            obsts_scenario;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();
        if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
          return py::list();

        aligned_vector<int> result(traj_in.size() + 2);

        collision::TrajectoryRequestCollisionTime req;
        req.broadphase_class = collision::BC_FCL;
        req.enable_verification = enable_verification;
        req.num_cells = num_cells;
        req.fcl_broadphase_type = broadphase_type;
        req.fcl_num_buckets = num_buckets;

        collision::TrajectoryQueryResultCollisionTime res(&result);

        int err = collision::trajectory_queries::
            trajectories_collision_time_dynamic_obstacles(
                traj_in, obsts_scenario, res, req);

        py::list ret_list;

        postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

        return ret_list;
      });

  mutil.def(
      "obb_enclosure_polygons_static",
      [](collision::ShapeGroup &sg_polygons, collision::RectangleOBB &rect) {
        bool res = false;
        size_t err = collision::geometry_queries::test_polygon_enclosure(
            sg_polygons, rect, res);
        py::list ret_list;
        ret_list.append(res);
        ret_list.append(err != 0);
        return ret_list;
      });

  mutil.def(
      "trajectory_collision_static_obstacles",
      [](collision::ShapeGroup &obj, double r_x, double r_y,
         std::vector<std::array<double, 3>> trajectory) {
        /*!
        Checks if the given trajectory (moving OBB box)
        collides with any objects of the given ShapeGroup.

        The function returns the time index of the first collision or -1.


        \param[in] obj ShapeGroup with static obstacles
        \param[in] r_x half-length of the oriented bounding box
        \param[in] r_y half-width of the oriented bounding box
        \param[in] trajectory list of arrays of 3 double numbers: x, y and
        orientation angle in radians.


        */

        int collision_index = -1;

        if (trajectory.size() == 1) {
          const auto &traj_cur = trajectory[0];
          collision::RectangleOBBConstPtr rect_cur(new collision::RectangleOBB(
              r_x, r_y, traj_cur[2],
              Eigen::Vector2d(traj_cur[0], traj_cur[1])));
          if (obj.collide(*rect_cur)) {
            collision_index = 0;
          }
        } else if (trajectory.size()) {
          for (int cc1 = 0; cc1 < trajectory.size() - 1; cc1++) {
            const auto &traj_cur = trajectory[cc1];

            collision::RectangleOBBConstPtr rect_cur(
                new collision::RectangleOBB(
                    r_x, r_y, traj_cur[2],
                    Eigen::Vector2d(traj_cur[0], traj_cur[1])));

            if (obj.collide(*rect_cur)) {
              collision_index = cc1;
              break;
            }
          }
        }

        return collision_index;
      });

  mutil.def(
      "filter_trajectories_polygon_enclosure_first_timestep",
      [](py::list trajectories, collision::ShapeGroup &obj) {
        aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

        if (preprocess_input_tvobstacles(trajectories, traj_in))
          return py::list();

        bool error = false;

        py::list traj_out;

        int cc1 = 0;

        Eigen::MatrixXd traj_exported(trajectories.size(), 5);

        for (auto traj : traj_in) {
          auto rect_obj = traj->getFirstObstaclePtr();
          if (rect_obj->getCollisionObjectType() ==
              collision::OBJ_TYPE_OBB_BOX) {
            auto obb_obj_ptr =
                static_cast<const collision::RectangleOBB *>(rect_obj);

            traj_exported(cc1, 0) = obb_obj_ptr->r_x();
            traj_exported(cc1, 1) = obb_obj_ptr->r_y();
            traj_exported(cc1, 2) = obb_obj_ptr->orientation();
            auto center = obb_obj_ptr->center();
            traj_exported(cc1, 3) = center[0];
            traj_exported(cc1, 4) = center[1];
          } else if (rect_obj->getCollisionObjectType() ==
                     collision::OBJ_TYPE_AABB_BOX) {
            auto aabb_obj_ptr =
                static_cast<const collision::RectangleAABB *>(rect_obj);

            traj_exported(cc1, 0) = aabb_obj_ptr->r_x();
            traj_exported(cc1, 1) = aabb_obj_ptr->r_y();
            traj_exported(cc1, 2) = 0;
            auto center = aabb_obj_ptr->center();
            traj_exported(cc1, 3) = center[0];
            traj_exported(cc1, 4) = center[1];
          } else {
            throw std::invalid_argument(
                "only trajectories containing obb boxes are currently "
                "supported");
          }

          cc1++;
        }
        cc1 = 0;
        for (auto traj : traj_in) {
          int collision_index = -1;

          const collision::RectangleOBB rect_cur(
              traj_exported(cc1, 0), traj_exported(cc1, 1),
              traj_exported(cc1, 2),
              Eigen::Vector2d(traj_exported(cc1, 3), traj_exported(cc1, 4)));
          bool loc_within = false;

          int loc_err = collision::geometry_queries::test_polygon_enclosure(
              obj, rect_cur, loc_within);
          if (loc_err) {
            error = true;
          } else if (!loc_within) {
            collision_index = 0;
          }

          if (collision_index == -1) {
            traj_out.append(trajectories[cc1]);
          }
          cc1++;
        }

        if (error) throw std::invalid_argument("invalid polygon input");

        return traj_out;
      });

  mutil.def(
      "trajectory_enclosure_polygons_static",
      [](collision::ShapeGroup &obj, double r_x, double r_y,
         std::vector<std::array<double, 3>> trajectory) {
        int collision_index = -1;

        bool error = false;

        if (trajectory.size() == 1) {
          const auto &traj_cur = trajectory[0];
          const collision::RectangleOBB rect_cur(
              r_x, r_y, traj_cur[2], Eigen::Vector2d(traj_cur[0], traj_cur[1]));
          bool loc_within = false;

          int loc_err = collision::geometry_queries::test_polygon_enclosure(
              obj, rect_cur, loc_within);
          if (loc_err) {
            error = true;
          } else if (!loc_within) {
            collision_index = 0;
          }
        } else if (trajectory.size()) {
          for (int cc1 = 0; cc1 < trajectory.size() - 1; cc1++) {
            const auto &traj_cur = trajectory[cc1];

            const collision::RectangleOBB rect_cur(
                r_x, r_y, traj_cur[2],
                Eigen::Vector2d(traj_cur[0], traj_cur[1]));

            std::size_t loc_err = 0;
            bool loc_within = false;
            loc_err = collision::geometry_queries::test_polygon_enclosure(
                obj, rect_cur, loc_within);
            if (loc_err) {
              error = true;
              break;
            }
            if (!loc_within) {
              collision_index = cc1;
              break;
            }
          }
        }
        if (error) throw std::invalid_argument("invalid polygon input");

        return collision_index;
      });

  mutil.def("trajectory_preprocess_obb_sum",
            [](collision::TimeVariantCollisionObject &tvobj) {
              collision::TimeVariantCollisionObjectPtr obj_new;
              int res =
                  collision::trajectory_queries::trajectory_preprocess_obb_sum(
                      &tvobj, obj_new);
              py::list ret_list;
              if (res != 0 || !obj_new) {
                ret_list.append(tvobj.shared_from_this());
              } else {
                ret_list.append(obj_new);
              }

              ret_list.append(res != 0);

              return ret_list;
            });

  mutil.def("polygon_contours_build_rectangles", [](collision::ShapeGroup
                                                        &sg_polys,
                                                    double rect_width) {
    collision::ShapeGroupPtr sg_rects =
        std::make_shared<collision::ShapeGroup>();
    auto sg_rects_ptr = sg_rects.get();
    for (auto obj : sg_polys.unpack()) {
      if (obj->getCollisionObjectType() == collision::OBJ_TYPE_POLYGON) {
        auto poly_ptr = static_cast<const collision::Polygon *>(obj.get());
        auto verts = poly_ptr->getVertices();
        collision::geometry_queries::create_rectangles_obb_from_vertex_list(
            verts, sg_rects_ptr, rect_width);
        auto holes = poly_ptr->getHoleVertices();
        for (auto &hole : holes) {
          collision::geometry_queries::create_rectangles_obb_from_vertex_list(
              hole, sg_rects_ptr, rect_width);
        }
      } else
        throw std::invalid_argument(
            "the shape group for building rectangles can contain only "
            "collision::Polygon objects");
    }
    return sg_rects;
  });

  mutil.def(
      "lane_polygons_postprocess",
      [](collision::ShapeGroup &sg_polys, double buf_width, bool triangulate) {
        return collision::solvers::solverBoost::lane_polygons_postprocess(
            sg_polys, buf_width, triangulate);
      });

  mutil.def("convert_tristrips_to_triangles",
            [](py::list tri_strips, std::vector<double> bb, int axis) {
              collision::ShapeGroupPtr sg_triangles =
                  std::make_shared<collision::ShapeGroup>();

              auto sg_tri_ptr = sg_triangles.get();

              double x_c = (bb[0] + bb[1]) / 2;
              double y_c = (bb[2] + bb[3]) / 2;
              double diff_c = y_c - x_c;
              double sum_c = x_c + y_c;

              Eigen::Matrix2d rot;
              rot << 0, 1, -1, 0;

              Eigen::Vector2d trans(-diff_c, sum_c);
              auto transl = trans.replicate<1, 3>();

              Eigen::Matrix<double, 2, 3> vertices;

              for (auto tri_strip : tri_strips) {
                for (auto vert_list_it : tri_strip.cast<py::list>()) {
                  auto vert_list =
                      vert_list_it.cast<std::vector<std::array<double, 2>>>();
                  for (auto vert = vert_list.begin();
                       vert < vert_list.end() - 2; vert++) {
                    vertices.col(0) = Eigen::Vector2d((*vert)[0], (*vert)[1]);
                    vertices.col(1) =
                        Eigen::Vector2d((*(vert + 1))[0], (*(vert + 1))[1]);
                    vertices.col(2) =
                        Eigen::Vector2d((*(vert + 2))[0], (*(vert + 2))[1]);

                    if (axis == 1) {
                      vertices = rot * vertices + transl;
                    }
                    auto tri = std::make_shared<const collision::Triangle>(
                        vertices.col(0), vertices.col(1), vertices.col(2));
                    sg_tri_ptr->addToGroup(tri);
                  }
                }
              }

              return sg_triangles;
            });

  // Note: The function below is EXPERIMENTAL (alpha version).
  // The underlying implementations of the experimental tolerance verification
  // function (in FCL and libccd) may have bugs or crash for some object types.
  // Many pairs of shape types are not supported. The function is exported only
  // for test purposes, and its results should not be relied upon. Distance
  // queries are not implemented in the collision checker. FCL does not support
  // them for many shape types.

  mutil_experimental.def(
      "tolerance_negative_query",
      [](collision::CollisionObjectConstPtr obj1,
         collision::CollisionObjectConstPtr obj2, bool aabb_only = false,
         double computation_tolerance = 1e-6, double compare_tolerance = 1e-6) {
        collision::DistanceRequest req;
        req.dist_request_type = collision::DRT_TOLERANCE_NEG;
        if (aabb_only)
          req.dist_node_type = collision::DNT_AABB;
        else
          req.dist_node_type = collision::DNT_NARROWPHASE;
        req.computation_tolerance = computation_tolerance;
        req.compare_tolerance = compare_tolerance;

        collision::DistanceResult res;
        int ret = collision::distance(*obj1, *obj2, res, req);
        py::list ret_list;
        ret_list.append(res.getTolerancePassed());
        ret_list.append(ret != 0);
        return ret_list;
      });

#endif
  ;
}

#if ENABLE_SERIALIZER
py::tuple pickle_object_out(const collision::CollisionObject &obj) {
  py::list ret;
  std::string dumped_obj;
  std::ostringstream obj_dump;
  obj.serialize(obj_dump);
  dumped_obj = obj_dump.str();
  ret.append(py::cast(dumped_obj));
  return py::make_tuple(ret);
}
#endif

#if ENABLE_SERIALIZER

// void null_deleter(const collision::CollisionObject *) {}
collision::CollisionObjectPtr pickle_object_in(py::tuple t) {
  /* Create a new C++ instance */
  std::string str_in;
  if (t.size() != 1)
    throw std::invalid_argument("pickle error - invalid input");
  py::list list_in;
  list_in = py::object(t[0]);
  if (list_in.size() != 1)
    throw std::invalid_argument("pickle error - invalid input");
  str_in = list_in[0].cast<std::string>();
  std::istringstream stream_in(str_in);
  collision::CollisionObjectConstPtr c =
      collision::CollisionObject::deserialize(stream_in);
  if (c.get() == 0) {
    throw std::invalid_argument("pickle error - invalid input");
  } else {
    // const collision::CollisionObject* ptr=c.get();
    // c.reset((collision::CollisionObject*)0, null_deleter);
    // return const_cast<collision::CollisionObject*>(ptr);
    collision::CollisionObjectPtr res =
        std::const_pointer_cast<collision::CollisionObject>(c);
    return res;
  }
  return collision::CollisionObjectPtr(0);
}
#endif

void init_module_collision(py::module &m) {
  py::class_<collision::CollisionObject,
             std::shared_ptr<collision::CollisionObject>>(m, "CollisionObject")

      ;

  auto pyShape = py::class_<collision::Shape, collision::CollisionObject,
             std::shared_ptr<collision::Shape>>(m, "Shape")
      ;

  py::class_<collision::Point, collision::Shape,
             std::shared_ptr<collision::Point>>(m, "Point")
      .def(py::init([](double x, double y) {
             return new collision::Point(Eigen::Vector2d(x, y));
           }),
           py::arg("x"), py::arg("y"))
      .def("collide",
           [](std::shared_ptr<collision::Point> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("center",
           [](collision::Point &point) {
             Eigen::Vector2d tmp = point.center();
             std::vector<double> pos;
             pos.push_back(tmp(0));
             pos.push_back(tmp(1));
             return py::array(2, pos.data());
           })

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::Point>(
                pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::RectangleAABB, collision::Shape,
             std::shared_ptr<collision::RectangleAABB>>(m, "RectAABB")
      .def(py::init([](double r_x, double r_y, double x, double y) {
             return new collision::RectangleAABB(r_x, r_y,
                                                 Eigen::Vector2d(x, y));
           }),
           py::arg("half_width"), py::arg("half_height"), py::arg("center_x"),
           py::arg("center_y"))
      .def("collide",
           [](std::shared_ptr<collision::RectangleAABB> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def(
          "__str__",
          [](const std::shared_ptr<collision::RectangleAABB> &c) {
            return "<collision::RectangleAABB r_x=" + std::to_string(c->r_x()) +
                   " r_y=" + std::to_string(c->r_y()) + ">";
          })
      .def("set_all", &collision::RectangleAABB::set_all)
      .def("min_x",
           [](const std::shared_ptr<collision::RectangleAABB> &c) {
             Eigen::Vector2d tmp = c->min();
             return tmp(0);
           })
      .def("min_y",
           [](const std::shared_ptr<collision::RectangleAABB> &c) {
             Eigen::Vector2d tmp = c->min();
             return tmp(1);
           })
      .def("max_x",
           [](const std::shared_ptr<collision::RectangleAABB> &c) {
             Eigen::Vector2d tmp = c->max();
             return tmp(0);
           })
      .def("max_y",
           [](const std::shared_ptr<collision::RectangleAABB> &c) {
             Eigen::Vector2d tmp = c->max();
             return tmp(1);
           })
      .def_property_readonly("r_x", &collision::RectangleAABB::r_x)

      .def("draw",
           [](const std::shared_ptr<collision::RectangleAABB> &c,
              py::object renderer, py::object draw_params)
              {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_rectaabb");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::RectangleAABB>(
                pickle_object_in(t));
          }))
#endif

      ;

  // Deferred definition - required to ensure correct signature
  // See https://pybind11.readthedocs.io/en/latest/advanced/misc.html#avoiding-cpp-types-in-docstrings
  pyShape.def("getAABB", &collision::CollisionObjectEx::getAABB);

  py::class_<collision::RectangleOBB, collision::Shape,
             std::shared_ptr<collision::RectangleOBB>>(m, "RectOBB")
      .def(py::init(
               [](double r_x, double r_y, double angle, double x, double y) {
                 return new collision::RectangleOBB(r_x, r_y, angle,
                                                    Eigen::Vector2d(x, y));
               }),
           py::arg("width/2"), py::arg("height/2"), py::arg("orientation"),
           py::arg("center x"), py::arg("center y"))
      .def("collide",
           [](std::shared_ptr<collision::RectangleOBB> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("set_center",
           [](collision::RectangleOBB &rect, double x, double y) {
             rect.set_center(Eigen::Vector2d(x, y));
           })
      .def("merge",
           [](std::shared_ptr<collision::RectangleOBB> &cc,
              std::shared_ptr<collision::RectangleOBB> &co) {
             return std::static_pointer_cast<const collision::RectangleOBB>(
                 collision::geometry_queries::ccd_merge_entities(cc.get(),
                                                                 co.get()));
           })
      .def("local_x_axis",
           [](collision::RectangleOBB &rect) {
             Eigen::Vector2d tmp = rect.local_x_axis();
             std::vector<double> pos;
             pos.push_back(tmp(0));
             pos.push_back(tmp(1));
             return py::array(2, pos.data());
           })
      .def("local_y_axis",
           [](collision::RectangleOBB &rect) {
             Eigen::Vector2d tmp = rect.local_y_axis();
             std::vector<double> pos;
             pos.push_back(tmp(0));
             pos.push_back(tmp(1));
             return py::array(2, pos.data());
           })
      .def("orientation", &collision::RectangleOBB::orientation,
           "OBB orientation")
      .def("r_x", &collision::RectangleOBB::r_x,
           "Positive halfwidth extent of OBB along local x-axis")
      .def("r_y", &collision::RectangleOBB::r_y,
           "Positive halfwidth extent of OBB along local y-axis")
      .def("center",
           [](collision::RectangleOBB &rect) {
             Eigen::Vector2d tmp = rect.center();
             std::vector<double> pos;
             pos.push_back(tmp(0));
             pos.push_back(tmp(1));
             return py::array(2, pos.data());
           })
      .def("__str__",
           [](collision::RectangleOBB &c) {
             return "<collision::RectangleOBB r_x=" + std::to_string(c.r_x()) +
                    " r_y=" + std::to_string(c.r_y()) +
                    " center_x=" + std::to_string(c.center_x()) +
                    " center_y=" + std::to_string(c.center_y()) + ">";
           })

      .def("draw",
           [](const std::shared_ptr<collision::RectangleOBB> &c,
              py::object renderer, py::object draw_params
              ) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_rectobb");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::RectangleOBB>(
                pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::Triangle, collision::Shape,
             std::shared_ptr<collision::Triangle>>(m, "Triangle")
      .def(py::init([](double x1, double y1, double x2, double y2, double x3,
                       double y3) {
             return new collision::Triangle(Eigen::Vector2d(x1, y1),
                                            Eigen::Vector2d(x2, y2),
                                            Eigen::Vector2d(x3, y3));
           }),
           py::arg("x1"), py::arg("y1"), py::arg("x2"), py::arg("y2"),
           py::arg("x3"), py::arg("y3"))
      .def("collide",
           [](std::shared_ptr<collision::Triangle> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("vertices",
           [](collision::Triangle &obj) {
             py::list v1;
             v1.append(py::cast(obj.v1()[0]));
             v1.append(py::cast(obj.v1()[1]));
             py::list v2;
             v2.append(py::cast(obj.v2()[0]));
             v2.append(py::cast(obj.v2()[1]));
             py::list v3;
             v3.append(py::cast(obj.v3()[0]));
             v3.append(py::cast(obj.v3()[1]));

             py::list ret_list;
             ret_list.append(v1);
             ret_list.append(v2);
             ret_list.append(v3);
             return ret_list;
           })
      .def("__str__",
           [](collision::Triangle &c) {
             return "<collision::Triangle v1=" + std::to_string(c.v1()[0]) +
                    "/" + std::to_string(c.v1()[1]) +
                    " v2=" + std::to_string(c.v2()[0]) + "/" +
                    std::to_string(c.v2()[1]) +
                    " v3=" + std::to_string(c.v3()[0]) + "/" +
                    std::to_string(c.v3()[1]) + ">";
           })
      .def(
          "draw",
          [](const std::shared_ptr<collision::Triangle> &c, py::object renderer,
             py::object draw_params) {
            py::object pycrcc = py::module::import(
                "commonroad_dc.collision.visualization.drawing");
            py::object draw = pycrcc.attr("draw_collision_triangle");
            draw(c, renderer, draw_params);
          },
          py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::Triangle>(
                pickle_object_in(t));
          }))
#endif
      ;

  py::class_<collision::Sphere, collision::Shape,
             std::shared_ptr<collision::Sphere>>(m, "Circle")
      .def(py::init<double, double, double>(), py::arg("radius"),
           py::arg("center x"), py::arg("center y"))
      .def("collide",
           [](std::shared_ptr<collision::Sphere> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("r", &collision::Sphere::radius, "radius of the circle")
      .def("center",
           [](collision::Sphere &sphere) {
             Eigen::Vector2d tmp = sphere.center();
             std::vector<double> pos;
             pos.push_back(tmp(0));
             pos.push_back(tmp(1));
             return py::array(2, pos.data());
           })
      .def("x", &collision::Sphere::get_x, "x-coordinate of center")
      .def("y", &collision::Sphere::get_y, "y-coordinate of center")

      .def("draw",
           [](const std::shared_ptr<collision::Sphere> &c, py::object renderer,
              py::object draw_params) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_circle");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::Sphere>(
                pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::TimeVariantCollisionObject, collision::CollisionObject,
             std::shared_ptr<collision::TimeVariantCollisionObject>>(
      m, "TimeVariantCollisionObject")
      .def(py::init<int>(), py::arg("time_start_idx"))
      .def("collide",
           [](std::shared_ptr<collision::TimeVariantCollisionObject> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("append_obstacle",
           [](collision::TimeVariantCollisionObject &obj,
              std::shared_ptr<collision::CollisionObject> co) {
             obj.appendObstacle(co);
           })
      .def("time_start_idx",
           &collision::TimeVariantCollisionObject::time_start_idx)
      .def("time_end_idx", &collision::TimeVariantCollisionObject::time_end_idx)
      .def("obstacle_at_time",
           &collision::TimeVariantCollisionObject::getObstacleAtTime)

      .def("draw",
           [](const std::shared_ptr<collision::TimeVariantCollisionObject> &c,
              py::object renderer, py::object draw_params) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw =
                 pycrcc.attr("draw_collision_timevariantcollisionobject");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<
                collision::TimeVariantCollisionObject>(pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::ShapeGroup, collision::CollisionObject,
             std::shared_ptr<collision::ShapeGroup>>(m, "ShapeGroup")
      .def(py::init<>())
      .def("collide",
           [](std::shared_ptr<collision::ShapeGroup> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })

      .def("overlap",
           [](std::shared_ptr<collision::ShapeGroup> &cc,
              std::shared_ptr<collision::ShapeGroup> &co) {
             std::vector<std::pair<int, int>> idx = cc->overlap(*co);
             py::list ret;
             for (auto &i : idx) {
               ret.append(py::cast(i));
             }
             return ret;
           })
      .def("overlap_map",
           [](std::shared_ptr<collision::ShapeGroup> &cc,
              std::shared_ptr<collision::ShapeGroup> &co) {
             std::vector<std::set<int>> idx = cc->overlapMap(*co);
             py::dict ret;
             int key = 0;
             for (auto &i : idx) {
               py::set app1;
               for (auto &j : i) {
                 app1.add(py::cast(j));
               }
               ret[py::cast(key)] = app1;
               key++;
             }
             return ret;
           })
      .def("add_shape",
           [](collision::ShapeGroup &obj,
              std::shared_ptr<collision::Shape> co) { obj.addToGroup(co); })
      .def("size",
           [](collision::ShapeGroup &obj) {
             auto unpacked = obj.unpack();
             return unpacked.size();
           })
      .def("window_query", &collision::ShapeGroup::windowQuery)

      .def("unpack",
           [](collision::ShapeGroup &obj) {
             auto unpacked = obj.unpack();
             py::list ret_list;
             for (auto &i : unpacked) {
               ret_list.append(py::cast(i));
             }
             return ret_list;
           })

      .def("draw",
           [](const std::shared_ptr<collision::ShapeGroup> &c,
              py::object renderer, py::object draw_params) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_shapegroup");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::ShapeGroup>(
                pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::Polygon, collision::Shape,
             std::shared_ptr<collision::Polygon>>(m, "Polygon")
      .def(py::init([](std::vector<std::array<double, 2>> outer_boundary,
                       std::vector<std::vector<std::array<double, 2>>> holes,
                       py::list python_mesh_triangles) {
             std::vector<Eigen::Vector2d> vertices;
             std::vector<std::vector<Eigen::Vector2d>> hole_vertices;
             std::vector<collision::TriangleConstPtr> mesh_triangles;

             for (const auto &hole : holes) {
               std::vector<Eigen::Vector2d> hole_vert;

               for (const auto &hole_vt : hole) {
                 hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
               }
               hole_vertices.push_back(hole_vert);
             }

             for (const auto &vertex : outer_boundary) {
               vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
             }

             for (const auto &triangle : python_mesh_triangles) {
               mesh_triangles.push_back(
                   triangle.cast<collision::TriangleConstPtr>());
             }
             return new collision::Polygon(vertices, hole_vertices,
                                           mesh_triangles);
           }),
           py::arg("outer_boundary"), py::arg("holes"),
           py::arg("triangle mesh"))
#if ENABLE_TRIANGULATION

	  .def(py::init([](std::vector<std::array<double, 2>> outer_boundary, std::vector<std::vector<std::array<double, 2>>> holes) {
		std::vector<Eigen::Vector2d> vertices;
		for (const auto &vertex : outer_boundary) {
		  vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
		}

		std::vector<std::vector<Eigen::Vector2d>> hole_vertices;


		 for (const auto &hole : holes) {
		   std::vector<Eigen::Vector2d> hole_vert;

		   for (const auto &hole_vt : hole) {
			 hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
		   }
		   hole_vertices.push_back(hole_vert);
		 }

		return new collision::Polygon(
			vertices, hole_vertices, 0, collision::triangulation::TriangulationQuality());
	  }))

      .def(py::init([](std::vector<std::array<double, 2>> outer_boundary, std::vector<std::vector<std::array<double, 2>>> holes, int triangulation_method,
                       double mesh_quality) {
        std::vector<Eigen::Vector2d> vertices;

        /*
        if(triangulation_method==1)
        {
        	using namespace pybind11::literals;


            py::module_ np = py::module_::import("numpy");

    		py::object np_equal = np.attr("equal");
    		auto test0=np_equal(outer_boundary[0],outer_boundary[-1]);
    		auto locals = py::dict("test0"_a=test0,"outer_boundary"_a=outer_boundary);
    		py::exec(R"(
    			test1 = all(test0)
    		)", py::globals(), locals);
    		bool test1=locals["test1"].cast<bool>();
    		if(test1)
    		{
    			py::exec(R"(
    			    			verts = outer_boundary[:-1]
    			    		)", py::globals(), locals);
    		}
    		else
    		{
    			py::exec(R"(
    			                verts = outer_boundary
    			    	 )", py::globals(), locals);
    		}

        }
        */

        for (const auto &vertex : outer_boundary) {
          vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
        }
        std::vector<std::vector<Eigen::Vector2d>> hole_vertices;


		 for (const auto &hole : holes) {
		   std::vector<Eigen::Vector2d> hole_vert;

		   for (const auto &hole_vt : hole) {
			 hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
		   }
		   hole_vertices.push_back(hole_vert);
		 }

        return new collision::Polygon(
            vertices, hole_vertices, triangulation_method, collision::triangulation::TriangulationQuality(
                          mesh_quality));
      }))
#endif
      .def("collide",
           [](std::shared_ptr<collision::Polygon> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(*co);
           })
      .def("triangle_mesh",
           [](collision::Polygon &obj) {
             auto triangles = obj.getTriangleMesh();
             py::list ret_list;
             for (collision::TriangleConstPtr &i : triangles) {
               ret_list.append(i);
             }
             return ret_list;
           })
      .def("vertices",
           [](collision::Polygon &obj) {
             py::list vertices;
             for (const auto &vertex : obj.getVertices()) {
               py::list point;
               point.append(vertex(0));
               point.append(vertex(1));
               vertices.append(point);
             }
             return vertices;
           })
      .def("hole_vertices",
           [](collision::Polygon &obj) {
             py::list holes_list;
             for (const auto &hole : obj.getHoleVertices()) {
               py::list hole_vertices;
               for (const auto &vertex : hole) {
                 py::list point;
                 point.append(vertex(0));
                 point.append(vertex(1));
                 hole_vertices.append(point);
               }
               holes_list.append(hole_vertices);
             }
             return holes_list;
           })
      .def("__str__",
           [](collision::Polygon &c) {
             std::stringstream ss;
             ss << "<collision::Polygon vertices=";
             for (const auto &v : c.getVertices()) {
               ss << "(" << v(0) << "/" << v(1) << ") ";
             }
             ss << "\n";
             ss << ">";
             std::string s = ss.str();
             return ss.str();
           })

      .def("draw",
           [](const std::shared_ptr<collision::Polygon> &c, py::object renderer,
              py::object draw_params) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_polygon");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionObject &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return pickle_object_out(obj);
          },
          [](py::tuple t) {  // __setstate__
            return std::static_pointer_cast<collision::Polygon>(
                pickle_object_in(t));
          }))
#endif

      ;

  py::class_<collision::detail::OBBDenseTrajectoryBatch,
             std::shared_ptr<collision::detail::OBBDenseTrajectoryBatch>>(
      m, "OBBTrajectoryBatch")
      .def(
          py::init([](Eigen::Ref<const RowMatrixXd> loaded_trajectories,
                      Eigen::Ref<const VectorXi> start_time_step,
                      double box_half_length, double box_half_width) {
            py::list ret;
            using OBB = collision::detail::OBB;
            std::vector<OBB, Eigen::aligned_allocator<OBB>> obb1_fast_container;
            obb1_fast_container.reserve(int(loaded_trajectories.rows() *
                                            loaded_trajectories.cols() / 3));

            // std::vector<std::unique_ptr<const collision::detail:>> objects;
            if (loaded_trajectories.rows() != start_time_step.size())
              throw std::runtime_error("Incompatible buffer dimension!");
            if (loaded_trajectories.cols() % 3 != 0)
              throw std::runtime_error("Incompatible buffer dimension!");
            int traj_length = int(loaded_trajectories.cols() / 3);
            for (int traj_ind = 0; traj_ind < loaded_trajectories.rows();
                 traj_ind++) {
              for (int idx = 0; idx < traj_length; idx++) {
                int offset = 3 * idx;
                double angle = loaded_trajectories(traj_ind, offset + 2);
                Eigen::Matrix2d local_axes;
                double cos_angle = cos(angle);
                double sin_angle = sin(angle);
                local_axes << cos_angle, -1 * sin_angle, sin_angle, cos_angle;

                obb1_fast_container.emplace_back(
                    local_axes,
                    Eigen::Vector2d(box_half_length, box_half_width),
                    Eigen::Vector2d(loaded_trajectories(traj_ind, offset + 0),
                                    loaded_trajectories(traj_ind, offset + 1)));
              }
            }

            auto traj_batch = new collision::detail::OBBDenseTrajectoryBatch(
                obb1_fast_container, traj_length, start_time_step);
            return traj_batch;
          }),
          "See 05_collision_checks_dynamic_obstacles.ipynb for an "
          "initialization example.")
      .def("preprocess_",
           &collision::detail::OBBDenseTrajectoryBatch::preprocess_inplace,
           "Preprocesses each trajectory using OBB sum hull (for continuous "
           "collision detection). The occupancies of the OBB boxes for two "
           "subsequent states are overapproximated with a tightly fitting OBB "
           "box. It is an in-place operation.")
      .def("to_tvobstacle",
           [](std::shared_ptr<collision::detail::OBBDenseTrajectoryBatch> &tb) {
             py::list ret;
             for (int traj_id = 0; traj_id < tb->size(); traj_id++) {
               int length = tb->length();

               int start_step = tb->get_start_step(traj_id);
               auto tvobstacle =
                   std::make_shared<collision::TimeVariantCollisionObject>(
                       start_step);

               for (int ts = start_step; ts < start_step + length; ts++) {
                 collision::detail::OBB obb;
                 collision::AABB aabb;
                 auto res = tb->get_object_at_time_ptr(traj_id, ts, obb, aabb);

                 if (res) return ret;

                 auto obb_col =
                     std::make_shared<const collision::RectangleOBB>(obb);
                 tvobstacle->appendObstacle(obb_col);
               }
               ret.append(tvobstacle);
             }
             return ret;
           },
           "Returns a list of TimeVariantCollisionObjects corresponding to the "
           "OBB trajectory batch.")
      /*
.def("computeAABB",
[](std::shared_ptr<collision::detail::OBBDenseTrajectoryBatch> &tb) {
return tb->computeAABBs();
})
*/
      ;

  py::class_<collision::CollisionChecker,
             std::shared_ptr<collision::CollisionChecker>>(m,
                                                           "CollisionChecker")
      .def(py::init<>())
      .def("number_of_obstacles",
           &collision::CollisionChecker::numberOfObstacles)
      .def("__str__",
           [](const std::shared_ptr<collision::CollisionChecker> &c) {
             std::ostringstream stream;
             c->print(stream);
             return "<collision::CollisionChecker\n" + stream.str() + ">";
           })
      .def("add_collision_object",
           &collision::CollisionChecker::addCollisionObject)
      .def("clone", &collision::CollisionChecker::clone_shared)

      .def("collide",
           [](const std::shared_ptr<collision::CollisionChecker> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             return cc->collide(co);
           })
      .def("raytrace",
           [](const std::shared_ptr<collision::CollisionChecker> &cc,
              double point1_x, double point1_y, double point2_x,
              double point2_y, bool join_intervals) {
             std::vector<collision::LineSegment> res_seg;
             bool intersects = cc->rayTrace(Eigen::Vector2d(point1_x, point1_y),
                                            Eigen::Vector2d(point2_x, point2_y),
                                            res_seg, join_intervals);
             py::list ret_list;
             for (auto &i : res_seg) {
               py::list ret_sublist;
               ret_sublist.append(py::cast(i.point1().x));
               ret_sublist.append(py::cast(i.point1().y));
               ret_sublist.append(py::cast(i.point2().x));
               ret_sublist.append(py::cast(i.point2().y));

               ret_list.append(ret_sublist);
             }

             return ret_list;
           })

      .def("find_all_colliding_objects",
           [](const std::shared_ptr<collision::CollisionChecker> &cc,
              std::shared_ptr<collision::CollisionObject> &co) {
             std::vector<collision::CollisionObjectConstPtr> obstacles;
             bool collides = cc->collide(co, obstacles);
             py::list ret_list;
             for (auto &i : obstacles) {
               ret_list.append(py::cast(i));
             }
             return ret_list;
           })
      .def("time_slice", &collision::CollisionChecker::timeSlice)
      .def("window_query", &collision::CollisionChecker::windowQuery)
      .def("obstacles",
           [](const std::shared_ptr<collision::CollisionChecker> &cc) {
             auto obstacles = cc->getObstacles();
             py::list ret_list;
             for (auto &o : obstacles) {
               ret_list.append(py::cast(o));
             }
             return ret_list;
           })
      .def("any_collides",
           [](const std::shared_ptr<collision::CollisionChecker> &cc,
              py::list py_obstacle_list) {
             std::vector<collision::CollisionObjectConstPtr>
                 collision_object_list;
             for (const auto &item : py_obstacle_list) {
               if (cc->collide(
                       item.cast<collision::CollisionObjectConstPtr>())) {
                 return true;
               }
             }
             return false;
           })
      .def("draw",
           [](const std::shared_ptr<collision::CollisionChecker> &c,
              py::object renderer, py::object draw_params) {
             py::object pycrcc = py::module::import(
                 "commonroad_dc.collision.visualization.drawing");
             py::object draw = pycrcc.attr("draw_collision_collisionchecker");
             draw(c, renderer, draw_params);
           },
           py::arg("renderer"), py::arg("draw_params") = py::none())

#if ENABLE_SERIALIZER
      .def(py::pickle(
          [](const collision::CollisionChecker &obj) {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            py::list ret;
            std::string dumped_obj;
            std::ostringstream obj_dump;
            obj.serialize(obj_dump);
            dumped_obj = obj_dump.str();
            ret.append(py::cast(dumped_obj));
            return py::make_tuple(ret);
          },
          [](py::tuple t) {  // __setstate__
                             /* Create a new C++ instance */
            std::string str_in;
            if (t.size() != 1)
              throw std::invalid_argument("pickle error - invalid input");
            py::list list_in;
            list_in = py::object(t[0]);
            if (list_in.size() != 1)
              throw std::invalid_argument("pickle error - invalid input");
            str_in = list_in[0].cast<std::string>();
            std::istringstream stream_in(str_in);
            collision::CollisionCheckerConstPtr c =
                collision::CollisionChecker::deserialize(stream_in);
            if (c.get() == 0) {
              throw std::invalid_argument("pickle error - invalid input");
            } else {
              collision::CollisionCheckerPtr res =
                  std::const_pointer_cast<collision::CollisionChecker>(c);
              return res;
            }
          }))

#endif
      ;
}
