#pragma once

#include "collision/collision_object.h"

#include <nanobind/nanobind.h>

#if ENABLE_SERIALIZER
#include <string>
#endif

void bind_all_shapes(const nanobind::module_ &module);

auto bind_shape(const nanobind::module_ &module);

auto bind_point(const nanobind::module_ &module);

auto bind_rectangle_aabb(const nanobind::module_ &module);

auto bind_rectangle_obb(const nanobind::module_ &module);

auto bind_triangle(const nanobind::module_ &module);

auto bind_sphere(const nanobind::module_ &module);

auto bind_tvo(const nanobind::module_ &module);

auto bind_shape_group(const nanobind::module_ &module);

auto bind_polygon(const nanobind::module_ &module);

#if ENABLE_SERIALIZER
std::string pickle_object_out(const collision::CollisionObject &obj);

collision::CollisionObjectPtr pickle_object_in(const std::string &str_in);

template<typename T, typename = std::enable_if_t<std::is_base_of_v<collision::CollisionObject, T> >, typename... U>
void add_pickling(nanobind::class_<T, U...> &shape_class) {
    shape_class
            .def("__getstate__",
                 [](const collision::CollisionObject &obj) {
                     return pickle_object_out(obj);
                 })
            .def("__setstate__",
                 [](T &obj, const std::string &str_in) {
                     auto unpickled = std::dynamic_pointer_cast<T>(pickle_object_in(str_in));
                     // We can safely move the data out of the shared_ptr, since no one else has access to it
                     new(&obj) T(std::move(*unpickled));
                     // reset the shared_ptr to avoid dangling references
                     unpickled.reset();
                 });
}
#endif
