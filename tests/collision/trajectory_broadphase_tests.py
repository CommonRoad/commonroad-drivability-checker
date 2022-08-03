from random_object_creator import RandomObjectCreator
import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries

import commonroad_dc.pycrcc as pycrcc
import pickle
import os

creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)

# create random AABB boxes

num_boxes = 10

iter = 0


def dump_broadphase_failure(obj1, obj2, extension):
    i = 0
    while os.path.exists("dumps/broadphase_dump" + extension + "_1_%s.xml" % i):
        i += 1
    with open("dumps/broadphase_dump" + extension + "_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/broadphase_dump" + extension + "_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def verify_tolerance(obj1, obj2):
    tol_res = [True]
    try:
        tol_res = pycrcc.Util.experimental.tolerance_negative_query(obj1, obj2, False, 1e-10, 1e-6)
    except(Exception):
        pass

    return tol_res[0]

iter_max = 5000
while iter <= iter_max:
    iter = iter + 1
    sg_boxes = pycrcc.ShapeGroup()
    for i in range(num_boxes):
        aabb = creator.create_random_aabb()
        sg_boxes.add_shape(aabb)
    aabb_2 = creator.create_random_aabb()
    tvobstacle = pycrcc.TimeVariantCollisionObject(0)
    tvobstacle.append_obstacle(aabb_2)
    try:
        res_fcl = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle], static_obstacles=sg_boxes,
                                                                             method='fcl', enable_verification=True)
        res_fcl = (res_fcl[0] != -1)

        res_box2d = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle], static_obstacles=sg_boxes,
                                                                               method='box2d', enable_verification=True)
        res_box2d = (res_box2d[0] != -1)

        res_grid = trajectory_queries.trajectories_collision_static_obstacles([tvobstacle], static_obstacles=sg_boxes,
                                                                              method='grid', enable_verification=True)
        res_grid = (res_grid[0] != -1)
    except(Exception):
        print("Runtime verification exception")
        dump_broadphase_failure(aabb_2, sg_boxes, 'runtime')
        res = verify_tolerance(aabb_2, sg_boxes)
        if res == False:
            print('potential broadphase failure detected')

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
                                                                                 method='fcl', enable_verification=True)
        except(Exception):
            pass

        if True:
            # dump_broadphase_failure(aabb_2,sg_boxes, 'traj') # dump all suspicious cases
            res = verify_tolerance(aabb_2, sg_boxes)
            if res == False:
                print('potential broadphase failure detected')
                dump_broadphase_failure(aabb_2, sg_boxes, 'traj')  # dump the case confirmed by tolerance verification
        if (res_grid == res_box2d):

            print("fcl trajectory query result differ, may be normal behavior")

        else:
            print("grid or box2d trajectory query result differ")
    if res_sg != res_gt:
        print("fcl sg results differ, possibly touching AABBs")

        if True:  # dump all suspicious cases
            dump_broadphase_failure(aabb_2, sg_boxes, 'fcl_gt')
            res = verify_tolerance(aabb_2, sg_boxes)
            if res == False:
                print('potential broadphase failure detected')

    if iter % 10000 == 0:
        print(iter)
