if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator
import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries
import commonroad_dc.pycrcc as pycrcc
import pickle
import os
from tqdm import tqdm

creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)

# create random AABB boxes

num_boxes = 10

iter = 0


def dump_broadphase_failure(obj1, obj2, extension):
    i = 0
    while os.path.exists("dumps/trajectory_broadphase_dump" + extension + "_1_%s.xml" % i):
        i += 1
    with open("dumps/trajectory_broadphase_dump" + extension + "_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/trajectory_broadphase_dump" + extension + "_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def verify_tolerance(obj1, obj2):
    tol_res = True
    try:
        tol_res = pycrcc.Test.is_borderline_case_aabb_tvobstacle_support(obj1, obj2)
    except(Exception):
        pass

    return tol_res


def run_test():
    os.makedirs("dumps", exist_ok=True)
    iter_max = 10000
    has_error = False
    for iter in tqdm(range(iter_max)):
        sg_boxes = pycrcc.ShapeGroup()
        for i in range(num_boxes):
            aabb = creator.create_random_aabb()
            sg_boxes.add_shape(aabb)
        aabb_2 = creator.create_random_aabb()
        tvobstacle = pycrcc.TimeVariantCollisionObject(0)
        tvobstacle.append_obstacle(aabb_2)
        try:
            res_fcl = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                 static_obstacles=sg_boxes,
                                                                                 method='fcl', enable_verification=True)
            res_fcl = (res_fcl[0] != -1)

            res_box2d = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                   static_obstacles=sg_boxes,
                                                                                   method='box2d',
                                                                                   enable_verification=True)
            res_box2d = (res_box2d[0] != -1)

            res_grid = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                  static_obstacles=sg_boxes,
                                                                                  method='grid',
                                                                                  enable_verification=True)
            res_grid = (res_grid[0] != -1)
        except(Exception):
            print("trajectory_broadphase_tests: runtime verification exception")
            dump_broadphase_failure(aabb_2, sg_boxes, 'runtime')
            res = verify_tolerance(aabb_2, sg_boxes)
            if res == False:
                print('trajectory_broadphase_tests: potential broadphase failure detected')
                has_error = True

        res_gt = False

        for el in sg_boxes.unpack():
            if (el.collide(aabb_2)):
                res_gt = True
                break

        res_sg = sg_boxes.collide(aabb_2)
        if res_fcl != res_gt or res_box2d != res_gt or res_grid != res_gt:
            try:
                res_grid = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                      static_obstacles=sg_boxes,
                                                                                      method='grid',
                                                                                      enable_verification=True)
                res_box2d = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                       static_obstacles=sg_boxes,
                                                                                       method='box2d',
                                                                                       enable_verification=True)
                res_fcl = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle],
                                                                                     static_obstacles=sg_boxes,
                                                                                     method='fcl',
                                                                                     enable_verification=True)
            except(Exception):
                pass

            if True:
                # dump_broadphase_failure(aabb_2,sg_boxes, 'traj') # dump all suspicious cases
                res = verify_tolerance(aabb_2, sg_boxes)
                if res == False:
                    print('trajectory_broadphase_tests: potential broadphase failure detected')
                    dump_broadphase_failure(aabb_2, sg_boxes,
                                            'traj')  # dump the case confirmed by tolerance verification
                    has_error = True
            if (res_grid == res_box2d):

                print("trajectory_broadphase_tests: fcl trajectory query result differ, may be normal behavior")

            else:
                print("trajectory_broadphase_tests: grid or box2d trajectory query result differ")
        if res_sg != res_gt:
            print("trajectory_broadphase_tests: fcl sg results differ, possibly touching AABBs")

            if True:  # dump all suspicious cases
                dump_broadphase_failure(aabb_2, sg_boxes, 'fcl_gt')
                res = verify_tolerance(aabb_2, sg_boxes)
                if res == False:
                    print('trajectory_broadphase_tests: potential broadphase failure detected')
                    has_error = True
    return has_error


if __name__ == "__main__":
    if run_test() == True:
        exit(1)
