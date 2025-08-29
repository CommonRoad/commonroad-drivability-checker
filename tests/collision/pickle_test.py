import commonroad_dc.pycrcc as pycrcc
import pickle
import numpy as np
from tqdm import tqdm

if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator


def run_test():
    tvo2 = pycrcc.TimeVariantCollisionObject(4)
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))  # time step 4
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 2))  # time step 5
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 3))  # time step 6
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 4))  # time step 7
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 5))  # time step 8
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 6))  # time step 9
    tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 7))  # time step 10
    i = 0

    creat = RandomObjectCreator(-20, 20, -20, 20, 500, 500)
    i_max = 10000
    has_error = False
    for iter in tqdm(range(i_max)):
        cc = pycrcc.CollisionChecker()
        cc.add_collision_object(tvo2)

        cc_dump = pickle.dumps(cc)

        cc2 = pickle.loads(cc_dump)

        cc_dump2 = pickle.dumps(cc2)

        if (cc_dump != cc_dump2):
            print("pickling error")
            has_error = True

        obj_0 = creat.create_random_shape()

        obj_dump = pickle.dumps(obj_0)

        obj2 = pickle.loads(obj_dump)

        obj_dump2 = pickle.dumps(obj2)

        if (obj_dump != obj_dump2):
            print("pickling error")
            has_error = True
    return has_error


if __name__ == "__main__":
    if run_test() == True:
        exit(1)
