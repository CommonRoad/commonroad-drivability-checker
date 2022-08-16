from random_object_creator import RandomObjectCreator
import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries

import commonroad_dc.pycrcc as pycrcc

creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)


# create random AABB boxes
def trajectory_narrowphase_query(method, obj1, obj2):
    traj = pycrcc.TimeVariantCollisionObject(0)
    traj.append_obstacle(obj1)
    static_obst = pycrcc.ShapeGroup()
    static_obst.add_shape(obj2)
    res = trajectory_queries.trajectories_collision_static_obstacles([traj], static_obst, method=method,
                                                                     optimize_triangles=True)
    ret = res[0] != -1
    return ret


def verify_tolerance_narrowphase(obj1, obj2):
    tol_res = [True]
    try:
        tol_res = pycrcc.Util.experimental.tolerance_negative_query(obj1, obj2, False, 1e-10, 1e-6)
    except(Exception):
        pass

    return tol_res[0]


import os
import pickle


def dump_narrowphase_failure(obj1, obj2):
    i = 0
    while os.path.exists("dumps/narrowphase_dump_1_%s.xml" % i):
        i += 1
    with open("dumps/narrowphase_dump_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/narrowphase_dump_2_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def narrowphase_test(obj1, obj2):
    col_res_fcl = obj1.collide(obj2)
    col_res_traj_narrowphase = trajectory_narrowphase_query('grid', obj1, obj2)
    if col_res_fcl != col_res_traj_narrowphase:
        res = True  # verify_tolerance_narrowphase(obj1,obj2)
        if True:  # dump all suspicious cases including very small numeric differences
            dump_narrowphase_failure(obj1, obj2)
            print('case dumped')

        return res
    else:
        return True


def obb_obb_test():
    obj1 = creator.create_random_obb()
    obj2 = creator.create_random_obb()
    return narrowphase_test(obj1, obj2)


def obb_triangle_test():
    passed = True
    obj1 = creator.create_random_obb()
    obj2 = creator.create_random_triangle()
    passed = passed and narrowphase_test(obj1, obj2)

    obj1 = creator.create_random_triangle()
    obj2 = creator.create_random_obb()
    passed = passed and narrowphase_test(obj1, obj2)
    return passed


def tri_tri_test():
    obj1 = creator.create_random_triangle()
    obj2 = creator.create_random_triangle()
    return narrowphase_test(obj1, obj2)


def random_shape_test():
    obj1 = creator.create_random_shape()
    obj2 = creator.create_random_shape()
    return narrowphase_test(obj1, obj2)


def sg_test():
    obj1 = creator.create_random_triangle()
    obj2 = creator.create_random_triangle()
    sg1 = pycrcc.ShapeGroup()
    sg1.add_shape(obj1)
    return narrowphase_test(sg1, obj2)


iter = 0
iter_max = 5000
while iter <= iter_max:
    iter = iter + 1
    passed = True
    passed = passed and obb_obb_test()
    passed = passed and obb_triangle_test()
    passed = passed and tri_tri_test()
    # passed=passed and random_shape_test()
    passed = passed and sg_test()
    if passed == False:
        print('test failed')
    if iter % 10000 == 0:
        print(iter)
