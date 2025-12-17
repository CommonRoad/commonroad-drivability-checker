from commonroad_dc import pycrcc
import shapely
import os
import numpy as np
import pickle
from tqdm import tqdm

if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator

def construct_shapely_polygon_obb(obb):
    v1 = obb.center() - obb.r_x() * obb.local_x_axis() + obb.r_y() * obb.local_y_axis()
    v2 = obb.center() + obb.r_x() * obb.local_x_axis() + obb.r_y() * obb.local_y_axis()
    v3 = obb.center() + obb.r_x() * obb.local_x_axis() - obb.r_y() * obb.local_y_axis()
    v4 = obb.center() - obb.r_x() * obb.local_x_axis() - obb.r_y() * obb.local_y_axis()

    return shapely.Polygon([v1, v2, v3, v4])

def construct_shapely_polygon_aabb(aabb):
    v1 = np.asarray([aabb.min_x(), aabb.min_y()])
    v2 = np.asarray([aabb.max_x(), aabb.min_y()])
    v3 = np.asarray([aabb.max_x(), aabb.max_y()])
    v4 = np.asarray([aabb.min_x(), aabb.max_y()])

    return shapely.Polygon([v1, v2, v3, v4])

def construct_shapely_polygon_tri(tri):
    return shapely.Polygon(tri.vertices())

def construct_shapely_polygon_polygon(poly):
    return shapely.union_all([construct_shapely_polygon_tri(tri) for tri in poly.triangle_mesh()])

def construct_shapely_point(point):
    return shapely.Point(point.center())

def construct_shapely_circle(circle):
    return shapely.Point(circle.center()).buffer(circle.r())

def construct_shapely_polygon(obj):
    if obj.__class__.__name__ == "RectOBB":
        return construct_shapely_polygon_obb(obj)

    if obj.__class__.__name__ == "RectAABB":
        return construct_shapely_polygon_aabb(obj)

    if obj.__class__.__name__ == "Triangle":
        return construct_shapely_polygon_tri(obj)

    if obj.__class__.__name__ == "Circle":
        return construct_shapely_circle(obj)

    if obj.__class__.__name__ == "Polygon":
        return construct_shapely_polygon_polygon(obj)

    if obj.__class__.__name__ == "Point":
        return construct_shapely_point(obj)

    raise Exception("narrowphase type is not supported for the construction of shapely polygons")

def intersect_at_the_border(obj1, obj2, eps=1e-10):
    try:
        shape1 = construct_shapely_polygon(obj1)
        shape2 = construct_shapely_polygon(obj2)
        if (shape1.intersection(shape2).area <= eps and shape1.distance(shape2) <= eps):
            return True
        return False
    except Exception as e:
        print(e)
        return False


def dump_narrowphase_failure(obj1, obj2):
    i = 0
    while os.path.exists("dumps/collision_narrowphase_dump_1_%s.xml" % i):
        i += 1
    with open("dumps/collision_narrowphase_dump_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/collision_narrowphase_dump_2_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def run_test():
    print(os.getpid())
    print('performing random collision narrowphase tests')
    creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)
    loopc_max = 500000
    has_error = False
    for loopc in tqdm(range(loopc_max)):
        obj1 = creator.create_random_shape()
        obj2 = creator.create_random_shape()
        result_primitive = pycrcc.Test.collide(obj1, obj2, 2)
        result_fcl = pycrcc.Test.collide(obj1, obj2, 1)

        if(result_primitive!=result_fcl):
            print("suspected error narrowphase " + obj1.__class__.__name__ + " " + obj2.__class__.__name__)
            if intersect_at_the_border(obj1, obj2) == False:
                print("case dumped")
                dump_narrowphase_failure(obj1, obj2)
                has_error = True
    return has_error

if __name__ == "__main__":
    run_test()
