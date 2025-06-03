#include "pybind.h"

#include "pybind_shapes.h"
#include "pybind_util.h"

#include "collision/collision_checker.h"

#include "collision/solvers/boost/boost_geometry_queries.h"
#include "collision/solvers/boost/boost_helpers.h"

#include "collision/solvers/geometry_queries.h"
#include "collision/solvers/trajectory_queries.h"

#include "collision/solvers/accelerators/declarations.h"

#include <Eigen/Dense>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#if ENABLE_SERIALIZER
#include <nanobind/stl/string.h>
#endif

#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>


namespace nb = nanobind;
using namespace nanobind::literals;

template<typename T>
using aligned_vector = collision::aligned_vector<T>;

using Matrixd = Eigen::MatrixXd;
using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
using Scalard = Matrixd::Scalar;
using Scalari = VectorXi::Scalar;
using RowMatrixXd =
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

NB_MODULE(pycrcc, module) {
    export_collision(module);
    export_util(module);
}

void export_collision(const nb::module_ &module) {
    nb::class_<collision::CollisionObject>(module, "CollisionObject");

    bind_all_shapes(module);

    nb::class_<collision::detail::OBBDenseTrajectoryBatch>(module, "OBBTrajectoryBatch")
            .def("__init__",
                 [](collision::detail::OBBDenseTrajectoryBatch *obj, Eigen::Ref<const RowMatrixXd> loaded_trajectories,
                    Eigen::Ref<const VectorXi> start_time_step,
                    double box_half_length, double box_half_width) {
                     nb::list ret;
                     using OBB = collision::detail::OBB;
                     std::vector<OBB, Eigen::aligned_allocator<OBB> > obb1_fast_container;
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

                     new (obj) collision::detail::OBBDenseTrajectoryBatch(
                         obb1_fast_container, traj_length, start_time_step);
                 },
                 "See 05_collision_checks_dynamic_obstacles.ipynb for an "
                 "initialization example.")
            .def("preprocess_",
                 &collision::detail::OBBDenseTrajectoryBatch::preprocess_inplace,
                 "Preprocesses each trajectory using OBB sum hull (for continuous "
                 "collision detection). The occupancies of the OBB boxes for two "
                 "subsequent states are overapproximated with a tightly fitting OBB "
                 "box. It is an in-place operation.")
            .def("to_tvobstacle",
                 [](const collision::detail::OBBDenseTrajectoryBatch &tb) {
                     nb::list ret;
                     for (int traj_id = 0; traj_id < tb.size(); traj_id++) {
                         int length = tb.length();

                         int start_step = tb.get_start_step(traj_id);
                         auto tvobstacle =
                                 std::make_shared<collision::TimeVariantCollisionObject>(
                                     start_step);

                         for (int ts = start_step; ts < start_step + length; ts++) {
                             collision::detail::OBB obb;
                             collision::AABB aabb;
                             auto res = tb.get_object_at_time_ptr(traj_id, ts, obb, aabb);

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
                 "OBB trajectory batch.");

    nb::class_<collision::CollisionChecker>(module, "CollisionChecker")
            .def(nb::init())
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
                     cc->rayTrace(Eigen::Vector2d(point1_x, point1_y),
                                  Eigen::Vector2d(point2_x, point2_y),
                                  res_seg, join_intervals);
                     nb::list ret_list;
                     for (auto &i: res_seg) {
                         nb::list ret_sublist;
                         ret_sublist.append(i.point1().x);
                         ret_sublist.append(i.point1().y);
                         ret_sublist.append(i.point2().x);
                         ret_sublist.append(i.point2().y);

                         ret_list.append(ret_sublist);
                     }

                     return ret_list;
                 })

            .def("find_all_colliding_objects",
                 [](const std::shared_ptr<collision::CollisionChecker> &cc,
                    std::shared_ptr<collision::CollisionObject> &co) {
                     std::vector<collision::CollisionObjectConstPtr> obstacles;
                     cc->collide(co, obstacles);
                     return obstacles;
                 })
            .def("time_slice", &collision::CollisionChecker::timeSlice)
            .def("window_query", &collision::CollisionChecker::windowQuery)
            .def("obstacles", &collision::CollisionChecker::getObstacles)
            .def("any_collides",
                 [](const std::shared_ptr<collision::CollisionChecker> &cc,
                    const std::vector<collision::CollisionObjectConstPtr> &collision_object_list) {
                     for (const auto &item: collision_object_list) {
                         if (cc->collide(item)) {
                             return true;
                         }
                     }
                     return false;
                 })
            .def("draw",
                 [](const std::shared_ptr<collision::CollisionChecker> &c,
                    nb::object renderer, nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_collisionchecker");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none())

#if ENABLE_SERIALIZER
            .def("__getstate__",
                [](const collision::CollisionChecker &obj) {
                    // Return a string that fully encodes the state of the object
                    std::ostringstream obj_dump;
                    obj.serialize(obj_dump);
                    return obj_dump.str();
                })
            .def("__setstate__",
                [](collision::CollisionChecker &obj, const std::string &str_in) {
                    // Create a new C++ instance
                    std::istringstream stream_in(str_in);
                    collision::CollisionCheckerConstPtr c =
                            collision::CollisionChecker::deserialize(stream_in);
                    if (c == nullptr) {
                        throw std::invalid_argument("pickle error - invalid input");
                    }
                    // We can safely move the data out of the shared_ptr, since no one else has access to it
                    new (&obj) collision::CollisionChecker(std::move(*c));
                    // reset the shared_ptr to avoid dangling references
                    c.reset();
                })

#endif
            ;
}
