if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator
import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries
import commonroad_dc.pycrcc as pycrcc
import shapely
from tqdm import tqdm

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


import os
import pickle


def dump_narrowphase_failure(obj1, obj2):
    i = 0
    while os.path.exists("dumps/trajectory_narrowphase_dump_1_%s.xml" % i):
        i += 1
    with open("dumps/trajectory_narrowphase_dump_1_%s.xml" % i, 'wb') as f:
        pickle.dump(obj1, f)
    with open("dumps/trajectory_narrowphase_dump_2_%s.xml" % i, 'wb') as f:
        pickle.dump(obj2, f)


def construct_shapely_polygon_obb(obb):
    v1 = obb.center() - obb.r_x() * obb.local_x_axis() + obb.r_y() * obb.local_y_axis()
    v2 = obb.center() + obb.r_x() * obb.local_x_axis() + obb.r_y() * obb.local_y_axis()
    v3 = obb.center() + obb.r_x() * obb.local_x_axis() - obb.r_y() * obb.local_y_axis()
    v4 = obb.center() - obb.r_x() * obb.local_x_axis() - obb.r_y() * obb.local_y_axis()

    return shapely.Polygon([v1, v2, v3, v4])


def construct_shapely_polygon_tri(tri):
    return shapely.Polygon(tri.vertices())


def construct_shapely_polygon(obj):
    if obj.__class__.__name__ == "RectOBB":
        return construct_shapely_polygon_obb(obj)
    else:
        if obj.__class__.__name__ == "Triangle":
            return construct_shapely_polygon_tri(obj)
    raise Exception("narrowphase type is not supported for the construction of shapely polygons")


def intersect_at_the_border(obj1, obj2, eps=1e-12):
    poly1 = construct_shapely_polygon(obj1)
    poly2 = construct_shapely_polygon(obj2)
    if (poly1.intersection(poly2).area <= eps and poly1.distance(poly2) <= eps):
        return True
    return False


def narrowphase_test(obj1, obj2):
    col_res_fcl = obj1.collide(obj2)
    col_res_traj_narrowphase = trajectory_narrowphase_query('grid', obj1, obj2)
    if col_res_fcl != col_res_traj_narrowphase:
        if obj1.__class__.__name__ == "ShapeGroup":
            res = intersect_at_the_border(obj1.unpack()[0], obj2)
        else:
            res = intersect_at_the_border(obj1, obj2)
        if res == False:
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


def run_test():
    os.makedirs("dumps", exist_ok=True)
    iter_max = 10000
    has_error = False
    for iter in tqdm(range(iter_max)):
        passed = True
        ret = obb_obb_test()
        passed = passed and ret
        ret = obb_triangle_test()
        passed = passed and ret
        ret = tri_tri_test()
        passed = passed and ret
        ret = sg_test()
        passed = passed and ret
        if passed == False:
            print('trajectory_narrowphase_tests: test failed')
            has_error = True
    return has_error


if __name__ == "__main__":
    if run_test() == True:
        exit(1)
