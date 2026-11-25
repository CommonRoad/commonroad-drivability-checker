import numpy as np
import shapely
from tqdm import tqdm

import commonroad_dc.pycrcc as pycrcc

if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator

def construct_shapely_polygon_tri(tri):
    return shapely.Polygon(tri.vertices())

def is_triangle_valid(tri):
    area = 0.0
    verts = tri.vertices()
    v1 = np.asarray(verts[0])
    v2 = np.asarray(verts[1])
    v3 = np.asarray(verts[2])

    area = construct_shapely_polygon_tri(tri).area

    side1 = np.linalg.norm((v2 - v1))
    side2 = np.linalg.norm((v3 - v1))
    side3 = np.linalg.norm((v3 - v2))

    max_side = max(side1, side2, side3)
    min_side = min(side1, side2, side3)

    if ((min_side > 1e-10) and (area / max_side) > 1e-10):
        return True
    else:
        return False

def run_test_invalid_triangle():
    random_object_creator = creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)
    num_iters = 100000
    eps = 1e-10
    for iter in tqdm(range(num_iters)):
        tri1 = random_object_creator.create_random_invalid_triangle_normal()
        tri2 = random_object_creator.create_random_invalid_triangle_side()
        tri3 = random_object_creator.create_random_valid_triangle()
        tri4 = random_object_creator.create_random_valid_triangle()

        if is_triangle_valid(tri1) == True:
            return True
        if is_triangle_valid(tri2) == True:
            return True
        if is_triangle_valid(tri3) == False:
            return True
        if is_triangle_valid(tri4) == False:
            return True
        """
        if tri1.is_valid() == True:
            return True
        if tri2.is_valid() == True:
            return True
        if tri3.is_valid() == False:
            return True
        if tri4.is_valid() == False:
            return True
        """
        if tri1.collide(tri2) == True:
            return True
        if tri2.collide(tri1) == True:
            return True
        if tri1.collide(tri3) == True:
            return True

        tri3_poly = construct_shapely_polygon_tri(tri3)
        tri4_poly = construct_shapely_polygon_tri(tri4)

        if (tri3.collide(tri4) != tri3_poly.intersects(tri4_poly)):
            if ((tri3_poly.intersection(tri4_poly).area <= eps and tri3_poly.distance(tri4_poly) <= eps) == False):
                return True

    return False


if __name__ == "__main__":
    if run_test_invalid_triangle() == True:
        exit(1)