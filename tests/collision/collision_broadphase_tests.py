if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator
import commonroad_dc.pycrcc as pycrcc

import os
import pickle
import random
from tqdm import tqdm
import numpy as np

creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)


def test_collision_checker():
    has_error = False
    cc1 = pycrcc.CollisionChecker()
    objs = list()
    for i in range(random.randint(0, 8)):
        obj = creator.create_random_object()
        objs.append(obj)

    for obj in objs:
        cc1.add_collision_object(obj)

    obj2 = creator.create_random_object()
    if pycrcc.Test.run_test_cc_collide_obj(cc1, obj2) == False:
        print("collision_broadphase_tests: run_test_cc_collide_obj failed")
        pycrcc.Test.run_test_cc_collide_obj(cc1, obj2)
        dump_broadphase_failure(cc1, obj2)
        has_error = True

    if pycrcc.Test.run_test_cc_get_colliding_obstacle(cc1, obj2) == False:
        pycrcc.Test.run_test_cc_get_colliding_obstacle(cc1, obj2)
        print("collision_broadphase_tests: run_test_cc_get_colliding_obstacle failed")
        dump_broadphase_failure(cc1, obj2)
        has_error = True

    if pycrcc.Test.run_test_cc_get_colliding_obstacles(cc1, obj2) == False:
        pycrcc.Test.run_test_cc_get_colliding_obstacles(cc1, obj2)
        print("collision_broadphase_tests: run_test_cc_get_colliding_obstacles failed")
        dump_broadphase_failure(cc1, obj2)
        has_error = True


def test_shape_group():
    has_error = False
    obj2 = creator.create_random_shape()
    sg1 = creator.create_random_shape_group(random.randint(0, 8))

    if pycrcc.Test.run_test_shapegroup_collide_obj(sg1, obj2) == False:
        print("collision_broadphase_tests: run_test_shapegroup_collide_obj failed")
        pycrcc.Test.run_test_shapegroup_collide_obj(sg1, obj2)
        dump_broadphase_failure(sg1, obj2)
        has_error = True

    sg2 = creator.create_random_shape_group(random.randint(0, 8))

    if pycrcc.Test.run_test_shapegroup_collide_sg(sg1, sg2) == False:
        print("collision_broadphase_tests: run_test_shapegroup_collide_sg failed")
        pycrcc.Test.run_test_shapegroup_collide_sg(sg1, sg2)
        dump_broadphase_failure(sg1, sg2)
        has_error = True
    return has_error


def dump_broadphase_failure(obj1, obj2):
    i = 0
    while os.path.exists("dumps/broadphase_dump_1_%s.xml" % i):
        i += 1
    with open("dumps/collision_broadphase_dump_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/collision_broadphase_dump_2_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def run_test():
    os.makedirs("dumps", exist_ok=True)
    iter = 0
    iter_max = 10000
    has_error = False
    for iter in tqdm(range(iter_max)):
        if (test_collision_checker() == True):
            has_error = True
        if (test_shape_group() == True):
            has_error = True
    return has_error


if __name__ == "__main__":
    ret = run_test()
    if ret == True:
        exit(1)
