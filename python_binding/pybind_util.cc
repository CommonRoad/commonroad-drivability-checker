#include "pybind_util.h"

#include "collision/collision_checker.h"
#include "collision/narrowphase/point.h"
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/triangle.h"
#include "collision/shape_group.h"
#include "collision/time_variant_collision_object.h"
#include "collision/solvers/boost/boost_geometry_queries.h"
#include "collision/solvers/boost/boost_helpers.h"
#include "collision/solvers/distance_queries.h"
#include "collision/solvers/geometry_queries.h"
#include "collision/solvers/trajectory_queries.h"
#include "collision/solvers/accelerators/declarations.h"

#include <Eigen/Dense>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <iostream>
#include <stdexcept>
#include <vector>

namespace nb = nanobind;

template<typename T>
using aligned_vector = collision::aligned_vector<T>;

inline int preprocess_input_tvobstacles(
    const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &obstacles_in,
    aligned_vector<const collision::TimeVariantCollisionObject *> &
    obstacles_out) {
    for (const auto &el: obstacles_in) {
        obstacles_out.push_back(el.get());
    }
    return obstacles_out.size() == 0;
}

inline int postprocess_dynamic_collision_result(aligned_vector<int> &result,
                                                int num_trajectories,
                                                nb::list &ret_list) {
    int num_colliding = 0;


    for (int cur_obst = 0; cur_obst < num_trajectories; cur_obst++) {
        ret_list.append(result[cur_obst]);
    }

    ret_list.append(*(result.end() - 2));
    ret_list.append(*(result.end() - 1));
    return 0;
}

void export_util(nb::module_ &module) {
    auto mutil = module.def_submodule(
        "Util", "Util is a submodule of pycrcc containing auxiliary functions");

    auto mutil_deprecated = mutil.def_submodule(
        "deprecated", "Deprecated utility functions submodule");
    auto mutil_experimental = mutil.def_submodule(
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
        [](const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, const collision::ShapeGroup &static_obstacles,
           bool enable_verification, int broadphase_type, int num_cells,
           int num_buckets) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();

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

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "trajectories_collision_static_obstacles_grid",
        [](const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, const collision::ShapeGroup &static_obstacles,
           bool enable_verification, bool auto_orientation, int num_cells,
           bool optimize_triangles) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();

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

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "trajectories_collision_static_obstacles_box2d",
        [](const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, const collision::ShapeGroup &static_obstacles,
           bool enable_verification) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();

            aligned_vector<int> result(traj_in.size() + 2);
            collision::TrajectoryRequestCollisionTime req;
            req.broadphase_class = collision::BC_BOX2D;
            req.enable_verification = enable_verification;
            collision::TrajectoryQueryResultCollisionTime res(&result);
            collision::trajectory_queries::
                    trajectories_collision_time_static_obstacles(
                        traj_in, static_obstacles, res, req);

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });
    mutil.def(
        "trajectories_enclosure_polygons_static_grid",
        [](const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories,
           const collision::ShapeGroup &static_obstacles_polygons,
           bool enable_verification, int num_cells) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();

            aligned_vector<int> result(traj_in.size() + 2);

            collision::TrajectoryRequestCollisionTime req;
            req.broadphase_class = collision::BC_UNIFORMGRID;
            req.enable_verification = enable_verification;
            req.num_cells = num_cells;
            collision::TrajectoryQueryResultCollisionTime res(&result);

            collision::trajectory_queries::
                    trajectories_enclosure_time_polygons_static(
                        traj_in, static_obstacles_polygons, res, req);

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "trajectories_collision_dynamic_obstacles_grid",
        [](aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &dynamic_obstacles,
           bool enable_verification, int num_cells) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            aligned_vector<const collision::TimeVariantCollisionObject *>
                    obsts_scenario;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();
            if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
                return nb::list();

            aligned_vector<int> result(traj_in.size() + 2);

            collision::TrajectoryRequestCollisionTime req;
            req.broadphase_class = collision::BC_UNIFORMGRID;
            req.enable_verification = enable_verification;
            req.num_cells = num_cells;
            collision::TrajectoryQueryResultCollisionTime res(&result);

            int err = collision::trajectory_queries::
                    trajectories_collision_time_dynamic_obstacles(
                        traj_in, obsts_scenario, res, req);

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "trajectories_collision_dynamic_obstacles_box2d",
        [](aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &dynamic_obstacles,
           bool enable_verification) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif

            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            aligned_vector<const collision::TimeVariantCollisionObject *>
                    obsts_scenario;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();
            if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
                return nb::list();

            aligned_vector<int> result(traj_in.size() + 2);

            collision::TrajectoryRequestCollisionTime req;
            req.broadphase_class = collision::BC_BOX2D;
            req.enable_verification = enable_verification;
            collision::TrajectoryQueryResultCollisionTime res(&result);

            int err = collision::trajectory_queries::
                    trajectories_collision_time_dynamic_obstacles(
                        traj_in, obsts_scenario, res, req);

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "trajectories_collision_dynamic_obstacles_fcl",
        [](aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &dynamic_obstacles,
           bool enable_verification, int broadphase_type, int num_cells,
           int num_buckets) {
#if TIME_PROFILE_ENABLED
            STACK_TIMER grid_total(TIMER_grid_static_total);
#endif
            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            aligned_vector<const collision::TimeVariantCollisionObject *>
                    obsts_scenario;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();
            if (preprocess_input_tvobstacles(dynamic_obstacles, obsts_scenario))
                return nb::list();

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

            nb::list ret_list;

            postprocess_dynamic_collision_result(result, traj_in.size(), ret_list);

            return ret_list;
        });

    mutil.def(
        "obb_enclosure_polygons_static",
        [](collision::ShapeGroup &sg_polygons, collision::RectangleOBB &rect) {
            bool res = false;
            size_t err = collision::geometry_queries::test_polygon_enclosure(
                sg_polygons, rect, res);
            nb::list ret_list;
            ret_list.append(res);
            ret_list.append(err != 0);
            return ret_list;
        });

    mutil.def(
        "trajectory_collision_static_obstacles",
        [](const collision::ShapeGroup &obj, double r_x, double r_y,
           const std::vector<std::array<double, 3> > &trajectory) {
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
        [](const aligned_vector<collision::TimeVariantCollisionObjectConstPtr> &trajectories, const collision::ShapeGroup &obj) {
            aligned_vector<const collision::TimeVariantCollisionObject *> traj_in;

            if (preprocess_input_tvobstacles(trajectories, traj_in))
                return nb::list();

            bool error = false;

            nb::list traj_out;

            int cc1 = 0;

            Eigen::MatrixXd traj_exported(trajectories.size(), 5);

            for (auto traj: traj_in) {
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
            for (auto traj: traj_in) {
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
        [](const collision::ShapeGroup &obj, double r_x, double r_y,
           const std::vector<std::array<double, 3> > &trajectory) {
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
              [](collision::TimeVariantCollisionObjectPtr tvobj) {
                  collision::TimeVariantCollisionObjectPtr obj_new;
                  int res =
                          collision::trajectory_queries::trajectory_preprocess_obb_sum(
                              tvobj.get(), obj_new);
                  nb::list ret_list;
                  if (res != 0 || !obj_new) {
                      ret_list.append(tvobj);
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
        for (auto obj: sg_polys.unpack()) {
            if (obj->getCollisionObjectType() == collision::OBJ_TYPE_POLYGON) {
                auto poly_ptr = static_cast<const collision::Polygon *>(obj.get());
                auto verts = poly_ptr->getVertices();
                collision::geometry_queries::create_rectangles_obb_from_vertex_list(
                    verts, sg_rects_ptr, rect_width);
                auto holes = poly_ptr->getHoleVertices();
                for (auto &hole: holes) {
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
              [](const std::vector<std::vector<std::vector<std::array<double, 2>>>> &tri_strips, std::vector<double> bb, int axis) {
                  auto sg_triangles = std::make_shared<collision::ShapeGroup>();

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

                  for (auto tri_strip: tri_strips) {
                      for (auto vert_list : tri_strip) {
                          for (auto vert = vert_list.begin();
                               vert < vert_list.end() - 2; ++vert) {
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
            nb::list ret_list;
            ret_list.append(res.getTolerancePassed());
            ret_list.append(ret != 0);
            return ret_list;
        });
}
