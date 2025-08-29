import commonroad_dc.pycrcc as pycrcc

if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator
from tqdm import tqdm


def run_test():
    print('performing random collision stress-tests')
    creat = RandomObjectCreator(-20, 20, -20, 20, 500, 500)
    loopc_max = 10000
    obj_0 = creat.create_random_shape()
    for iter in tqdm(range(loopc_max)):
        cc1 = pycrcc.CollisionChecker()
        objs = list()
        for i in range(30):
            obj = creat.create_random_object()
            objs.append(obj)
        for obj in objs:
            cc1.add_collision_object(obj)

        obj2 = creat.create_random_object()
        sg_new = pycrcc.ShapeGroup()
        sg_new.add_shape(obj_0)
        cc1.collide(obj2)
        obstacles = cc1.find_all_colliding_objects(obj2)

    print("done")


if __name__ == "__main__":
    run_test()
