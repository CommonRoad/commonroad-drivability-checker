import commonroad_dc.pycrcc as pycrcc
import numpy as np
from random_object_creator import RandomObjectCreator

from time import sleep

# import tracemalloc

# tracemalloc.start()

print('performing random collision stress-tests')
creat = RandomObjectCreator(-20, 20, -20, 20, 500, 500)
loopc = 0
loopc_max = 5000
obj_0 = creat.create_random_shape()
while (loopc <= loopc_max):
    loopc = loopc + 1
    if (loopc % 10 == 0):
        print(loopc)
        sleep(0.05)
    if (loopc % 1000 == 0):
        sleep(0.05)
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

    # snapshot = tracemalloc.take_snapshot()
    # top_stats = snapshot.statistics('lineno')

    # for stat in top_stats[:10]:
    #    print(stat)

    # cc1.find_all_colliding_objects(obj2)

print("done")
