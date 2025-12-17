import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries
import copy
import commonroad_dc.pycrcc as pycrcc


def run_test():
    tvobstacle = pycrcc.TimeVariantCollisionObject(0)

    tvobstacle.append_obstacle(pycrcc.ShapeGroup())

    obb = pycrcc.RectOBB(1, 1, 0, 0, 0)

    tvobstacle_2 = pycrcc.TimeVariantCollisionObject(0)

    tvobstacle_2.append_obstacle(obb)

    sg_boxes = pycrcc.ShapeGroup()

    traj = copy.deepcopy(tvobstacle)

    res_static = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle], static_obstacles=sg_boxes,
                                                                            method='grid', enable_verification=True)

    res_dynamic = trajectory_queries.trajectories_collision_dynamic_obstacles([tvobstacle], [traj], method='grid')

    res_poly = trajectory_queries.trajectories_enclosure_polygons_static([tvobstacle_2], sg_boxes)

    test_passed = True

    if (res_static != [-1]):
        print("trajectories_collision_static_obstacles: test_failed")
        test_passed = False

    if (res_dynamic != [-1]):
        print("trajectories_collision_dynamic_obstacles: test_failed")
        test_passed = False

    if (res_poly != [0]):
        print("trajectories_enclosure_polygons_static: test_failed")
        test_passed = False

    if (test_passed == True):
        print("trajectory_test_empty_sg: all checks passed")
    return test_passed == False


if __name__ == "__main__":
    if run_test() == True:
        exit(1)
