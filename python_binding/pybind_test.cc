#include "pybind_test.h"

#include "collision/collision_checker.h"
#include "collision/shape_group.h"
#include "collision/tests/broadphase_test2.h"
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

void export_test(nb::module_ &module) {
	using namespace collision::test;
	auto mtest = module.def_submodule("Test",
			"Test is a submodule of pycrcc containing test functions");

	mtest.def("run_test_shapegroup_collide_obj",
			[](collision::ShapeGroupConstPtr sg,
					collision::CollisionObjectConstPtr obj) {
				return ShapeGroupTest::run_test_collide(obj, sg.get());
			});

	mtest.def("run_test_shapegroup_collide_sg",
			[](collision::ShapeGroupConstPtr sg1,
					collision::ShapeGroupConstPtr sg2) {
				return ShapeGroupTest::run_test_collide(sg1.get(), sg2.get());
			});

	mtest.def("run_test_cc_collide_obj",
			[](std::shared_ptr<collision::CollisionChecker> cc,
					std::shared_ptr<collision::CollisionObject> &obj) {
				return CollisionCheckerTest::run_test_collide(obj, cc.get());
			});

	mtest.def("run_test_cc_get_colliding_obstacle",
			[](collision::CollisionCheckerConstPtr cc,
					collision::CollisionObjectConstPtr obj) {
				return CollisionCheckerTest::run_test_collide_obstacle(obj,
						cc.get());
			});
	mtest.def("run_test_cc_get_colliding_obstacles",
			[](collision::CollisionCheckerConstPtr cc,
					collision::CollisionObjectConstPtr obj) {
				return CollisionCheckerTest::run_test_collide_obstacles(obj,
						cc.get());
			});

}
