#include "pybind_shapes.h"

#include "collision/narrowphase/point.h"
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/sphere.h"
#include "collision/narrowphase/triangle.h"
#include "collision/shape_group.h"
#include "collision/solvers/geometry_queries.h"
#include "collision/time_variant_collision_object.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#if ENABLE_SERIALIZER
#include <nanobind/stl/string.h>
#endif


namespace nb = nanobind;
using namespace nanobind::literals;

auto bind_shape(const nb::module_ &module) {
    return nb::class_<collision::Shape, collision::CollisionObject>(module, "Shape");
}

auto bind_point(const nb::module_ &module) {
    return nb::class_<collision::Point, collision::Shape>(module, "Point")
            .def("__init__", [](collision::Point *obj, double x, double y) {
                     new(obj) collision::Point({x, y});
                 },
                 "x"_a, "y"_a)
            .def("collide",
                 [](const collision::Point &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("center", &collision::Point::center);
}

auto bind_rectangle_aabb(const nb::module_ &module) {
    return nb::class_<collision::RectangleAABB, collision::Shape>(module, "RectAABB")
            .def("__init__", [](collision::RectangleAABB *obj, double r_x, double r_y, double x, double y) {
                     new(obj) collision::RectangleAABB(r_x, r_y, {x, y});
                 },
                 "half_width"_a, "half_height"_a, "center_x"_a, "center_y"_a)
            .def("collide",
                 [](const collision::RectangleAABB &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def(
                "__str__",
                [](const collision::RectangleAABB &c) {
                    return "<collision::RectangleAABB r_x=" + std::to_string(c.r_x()) +
                           " r_y=" + std::to_string(c.r_y()) + ">";
                })
            .def("set_all", &collision::RectangleAABB::set_all)
            .def("min_x",
                 [](const collision::RectangleAABB &c) {
                     return c.min()(0);
                 })
            .def("min_y",
                 [](const collision::RectangleAABB &c) {
                     return c.min()(1);
                 })
            .def("max_x",
                 [](const collision::RectangleAABB &c) {
                     return c.max()(0);
                 })
            .def("max_y",
                 [](const collision::RectangleAABB &c) {
                     return c.max()(1);
                 })
            .def_prop_ro("r_x", &collision::RectangleAABB::r_x)
            .def("draw",
                 [](const collision::RectangleAABB &c,
                    nb::object renderer, nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_rectaabb");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_rectangle_obb(const nb::module_ &module) {
    return nb::class_<collision::RectangleOBB, collision::Shape>(module, "RectOBB")
            .def("__init__",
                 [](collision::RectangleOBB *obj, double r_x, double r_y, double angle, double x, double y) {
                     new(obj) collision::RectangleOBB(r_x, r_y, angle,
                                                      {x, y});
                 },
                 "width/2"_a, "height/2"_a, "orientation"_a, "center x"_a, "center y"_a)
            .def("collide",
                 [](const collision::RectangleOBB &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("set_center",
                 [](collision::RectangleOBB &rect, double x, double y) {
                     rect.set_center({x, y});
                 })
            .def("merge",
                 [](std::shared_ptr<collision::RectangleOBB> &cc,
                    std::shared_ptr<collision::RectangleOBB> &co) {
                     return std::dynamic_pointer_cast<const collision::RectangleOBB>(
                         collision::geometry_queries::ccd_merge_entities(cc.get(),
                                                                         co.get()));
                 })
            .def("local_x_axis", &collision::RectangleOBB::local_x_axis)
            .def("local_y_axis", &collision::RectangleOBB::local_y_axis)
            .def("orientation", &collision::RectangleOBB::orientation,
                 "OBB orientation")
            .def("r_x", &collision::RectangleOBB::r_x,
                 "Positive halfwidth extent of OBB along local x-axis")
            .def("r_y", &collision::RectangleOBB::r_y,
                 "Positive halfwidth extent of OBB along local y-axis")
            .def("center", &collision::RectangleOBB::center)
            .def("__str__",
                 [](const collision::RectangleOBB &c) {
                     return "<collision::RectangleOBB r_x=" + std::to_string(c.r_x()) +
                            " r_y=" + std::to_string(c.r_y()) +
                            " center_x=" + std::to_string(c.center_x()) +
                            " center_y=" + std::to_string(c.center_y()) + ">";
                 })

            .def("draw",
                 [](const collision::RectangleOBB &c,
                    nb::object renderer, nb::object draw_params
         ) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_rectobb");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_triangle(const nb::module_ &module) {
    return nb::class_<collision::Triangle, collision::Shape>(module, "Triangle")
            .def("__init__", [](collision::Triangle *obj, double x1, double y1, double x2, double y2, double x3,
                                double y3) {
                     new(obj) collision::Triangle({x1, y1}, {x2, y2}, {x3, y3});
                 },
                 nb::arg("x1"), nb::arg("y1"), nb::arg("x2"), nb::arg("y2"),
                 nb::arg("x3"), nb::arg("y3"))
            .def("collide",
                 [](const collision::Triangle &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("vertices",
                 [](const collision::Triangle &obj) {
                     nb::list v1;
                     v1.append(nb::cast(obj.v1()[0]));
                     v1.append(nb::cast(obj.v1()[1]));
                     nb::list v2;
                     v2.append(nb::cast(obj.v2()[0]));
                     v2.append(nb::cast(obj.v2()[1]));
                     nb::list v3;
                     v3.append(nb::cast(obj.v3()[0]));
                     v3.append(nb::cast(obj.v3()[1]));

                     nb::list ret_list;
                     ret_list.append(v1);
                     ret_list.append(v2);
                     ret_list.append(v3);
                     return ret_list;
                 })
            .def("__str__",
                 [](const collision::Triangle &c) {
                     return "<collision::Triangle v1=" + std::to_string(c.v1()[0]) +
                            "/" + std::to_string(c.v1()[1]) +
                            " v2=" + std::to_string(c.v2()[0]) + "/" +
                            std::to_string(c.v2()[1]) +
                            " v3=" + std::to_string(c.v3()[0]) + "/" +
                            std::to_string(c.v3()[1]) + ">";
                 })
            .def(
                "draw",
                [](const collision::Triangle &c, nb::object renderer,
                   nb::object draw_params) {
                    nb::object pycrcc = nb::module_::import_(
                        "commonroad_dc.collision.visualization.drawing");
                    nb::object draw = pycrcc.attr("draw_collision_triangle");
                    draw(c, renderer, draw_params);
                },
                "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_sphere(const nb::module_ &module) {
    return nb::class_<collision::Sphere, collision::Shape>(module, "Circle")
            .def(nb::init<double, double, double>(), "radius"_a, "center x"_a, "center y"_a)
            .def("collide",
                 [](const collision::Sphere &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("r", &collision::Sphere::radius, "radius of the circle")
            .def("center", &collision::Sphere::center, "center of the circle")
            .def("x", &collision::Sphere::get_x, "x-coordinate of center")
            .def("y", &collision::Sphere::get_y, "y-coordinate of center")
            .def("draw",
                 [](const collision::Sphere &c, nb::object renderer,
                    nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_circle");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_tvo(const nb::module_ &module) {
    return nb::class_<collision::TimeVariantCollisionObject, collision::CollisionObject>(
                module, "TimeVariantCollisionObject")
            .def(nb::init<int>(), nb::arg("time_start_idx"))
            .def("collide",
                 [](const collision::TimeVariantCollisionObject &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("append_obstacle", &collision::TimeVariantCollisionObject::appendObstacle)
            .def("time_start_idx", &collision::TimeVariantCollisionObject::time_start_idx)
            .def("time_end_idx", &collision::TimeVariantCollisionObject::time_end_idx)
            .def("obstacle_at_time", &collision::TimeVariantCollisionObject::getObstacleAtTime)
            .def("draw",
                 [](const collision::TimeVariantCollisionObject &c,
                    nb::object renderer, nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw =
                             pycrcc.attr("draw_collision_timevariantcollisionobject");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_shape_group(const nb::module_ &module) {
    return nb::class_<collision::ShapeGroup, collision::CollisionObject>(module, "ShapeGroup")
            .def(nb::init())
            .def("collide",
                 [](const collision::ShapeGroup &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("overlap", &collision::ShapeGroup::overlap)
            .def("overlap_map",
                 [](const collision::ShapeGroup &cc,
                    const collision::ShapeGroup &co) {
                     auto idx = cc.overlapMap(co);
                     nb::dict ret;
                     int key = 0;
                     for (auto &i: idx) {
                         nb::set app1;
                         for (auto &j: i) {
                             app1.add(nb::cast(j));
                         }
                         ret[nb::cast(key)] = app1;
                         key++;
                     }
                     return ret;
                 })
            .def("add_shape", &collision::ShapeGroup::addToGroup)
            .def("size",
                 [](const collision::ShapeGroup &obj) {
                     return obj.unpack().size();
                 })
            .def("window_query", &collision::ShapeGroup::windowQuery)

            .def("unpack", &collision::ShapeGroup::unpack)
            .def("draw",
                 [](const collision::ShapeGroup &c,
                    nb::object renderer, nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_shapegroup");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

auto bind_polygon(const nb::module_ &module) {
    return nb::class_<collision::Polygon, collision::Shape>(module, "Polygon")
            .def("__init__", [](collision::Polygon *obj, const std::vector<std::array<double, 2> > &outer_boundary,
                                const std::vector<std::vector<std::array<double, 2> > > &holes,
                                std::vector<collision::TriangleConstPtr> mesh_triangles) {
                     std::vector<Eigen::Vector2d> vertices;
                     std::vector<std::vector<Eigen::Vector2d> > hole_vertices;

                     for (const auto &hole: holes) {
                         std::vector<Eigen::Vector2d> hole_vert;

                         for (const auto &hole_vt: hole) {
                             hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
                         }
                         hole_vertices.push_back(hole_vert);
                     }

                     for (const auto &vertex: outer_boundary) {
                         vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
                     }

                     new(obj) collision::Polygon(vertices, hole_vertices,
                                                 mesh_triangles);
                 },
                 "outer_boundary"_a, "holes"_a, "triangle mesh"_a)
#if ENABLE_TRIANGULATION

            .def("__init__", [](collision::Polygon *obj, const std::vector<std::array<double, 2> > &outer_boundary,
                                const std::vector<std::vector<std::array<double, 2> > > &holes) {
                std::vector<Eigen::Vector2d> vertices;
                for (const auto &vertex: outer_boundary) {
                    vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
                }

                std::vector<std::vector<Eigen::Vector2d> > hole_vertices;


                for (const auto &hole: holes) {
                    std::vector<Eigen::Vector2d> hole_vert;

                    for (const auto &hole_vt: hole) {
                        hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
                    }
                    hole_vertices.push_back(hole_vert);
                }

                new(obj) collision::Polygon(
                    vertices, hole_vertices, 0, collision::triangulation::TriangulationQuality());
            })

            .def("__init__", [](collision::Polygon *obj, const std::vector<std::array<double, 2>> &outer_boundary,
                             const std::vector<std::vector<std::array<double, 2>>> &holes, int triangulation_method,
                             double mesh_quality) {
                std::vector<Eigen::Vector2d> vertices;

                for (const auto &vertex: outer_boundary) {
                    vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
                }
                std::vector<std::vector<Eigen::Vector2d> > hole_vertices;


                for (const auto &hole: holes) {
                    std::vector<Eigen::Vector2d> hole_vert;

                    for (const auto &hole_vt: hole) {
                        hole_vert.push_back(Eigen::Vector2d(hole_vt[0], hole_vt[1]));
                    }
                    hole_vertices.push_back(hole_vert);
                }

                new(obj) collision::Polygon(
                    vertices, hole_vertices, triangulation_method, collision::triangulation::TriangulationQuality(
                        mesh_quality));
            })
#endif
            .def("collide",
                 [](const collision::Polygon &cc,
                    const collision::CollisionObject &co) {
                     return cc.collide(co);
                 })
            .def("triangle_mesh", &collision::Polygon::getTriangleMesh)
            .def("vertices",
                 [](const collision::Polygon &obj) {
                     nb::list vertices;
                     for (const auto &vertex: obj.getVertices()) {
                         nb::list point;
                         point.append(vertex(0));
                         point.append(vertex(1));
                         vertices.append(point);
                     }
                     return vertices;
                 })
            .def("hole_vertices",
                 [](const collision::Polygon &obj) {
                     nb::list holes_list;
                     for (const auto &hole: obj.getHoleVertices()) {
                         nb::list hole_vertices;
                         for (const auto &vertex: hole) {
                             nb::list point;
                             point.append(vertex(0));
                             point.append(vertex(1));
                             hole_vertices.append(point);
                         }
                         holes_list.append(hole_vertices);
                     }
                     return holes_list;
                 })
            .def("__str__",
                 [](const collision::Polygon &c) {
                     std::stringstream ss;
                     ss << "<collision::Polygon vertices=";
                     for (const auto &v: c.getVertices()) {
                         ss << "(" << v(0) << "/" << v(1) << ") ";
                     }
                     ss << "\n";
                     ss << ">";
                     std::string s = ss.str();
                     return ss.str();
                 })

            .def("draw",
                 [](const collision::Polygon &c, nb::object renderer,
                    nb::object draw_params) {
                     nb::object pycrcc = nb::module_::import_(
                         "commonroad_dc.collision.visualization.drawing");
                     nb::object draw = pycrcc.attr("draw_collision_polygon");
                     draw(c, renderer, draw_params);
                 },
                 "renderer"_a, "draw_params"_a = nb::none());
}

void bind_all_shapes(const nb::module_ &module) {
    auto shape = bind_shape(module);
    auto point = bind_point(module);
    auto rectangle_aabb = bind_rectangle_aabb(module);
    auto rectangle_obb = bind_rectangle_obb(module);
    auto triangle = bind_triangle(module);
    auto sphere = bind_sphere(module);
    auto tvo = bind_tvo(module);
    auto shape_group = bind_shape_group(module);
    auto polygon = bind_polygon(module);

    // Deferred definition - required to ensure correct signature
    // See https://pybind11.readthedocs.io/en/latest/advanced/misc.html#avoiding-cpp-types-in-docstrings
    shape.def("getAABB", &collision::CollisionObjectEx::getAABB);

#if ENABLE_SERIALIZER
    // Add pickling support to all shapes
    add_pickling(point);
    add_pickling(rectangle_aabb);
    add_pickling(rectangle_obb);
    add_pickling(triangle);
    add_pickling(sphere);
    add_pickling(tvo);
    add_pickling(shape_group);
    add_pickling(polygon);
#endif
}

#if ENABLE_SERIALIZER
std::string pickle_object_out(const collision::CollisionObject &obj) {
    std::ostringstream obj_dump;
    obj.serialize(obj_dump);
    return obj_dump.str();
}

collision::CollisionObjectPtr pickle_object_in(const std::string &str_in) {
    std::istringstream stream_in(str_in);
    collision::CollisionObjectConstPtr c =
            collision::CollisionObject::deserialize(stream_in);
    if (c == nullptr) {
        throw std::invalid_argument("pickle error - invalid input");
    }
    return std::const_pointer_cast<collision::CollisionObject>(c);
}
#endif
