import numpy as np
from tqdm import tqdm
from shapely.geometry import MultiPoint

if __name__ == "__main__":
    from random_object_creator import RandomObjectCreator
else:
    from .random_object_creator import RandomObjectCreator

import commonroad_dc.pycrcc as pycrcc

def create_random_convex_polygon(num_points=15, bounds=(0, 100)):
    # 1. Generate a cloud of random x, y coordinates
    x = np.random.uniform(bounds[0], bounds[1], num_points)
    y = np.random.uniform(bounds[0], bounds[1], num_points)

    # 2. Use MultiPoint to find the outermost perimeter
    points = np.column_stack((x, y))
    point_cloud = MultiPoint(points)

    # 3. convex_hull automatically outputs a valid Polygon
    return point_cloud.convex_hull

def create_random_convex_polygon(num_points=15, bounds=(0, 100)):
    x = np.random.uniform(bounds[0], bounds[1], num_points)
    y = np.random.uniform(bounds[0], bounds[1], num_points)
    points = np.column_stack((x, y))
    point_cloud = MultiPoint(points)
    return point_cloud.convex_hull

def run_test_triangle_polygon_reordering():
    rnd_creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)

    iter_max = 10000
    has_error = False
    for iter in tqdm(range(iter_max)):

        p1 = rnd_creator.generate_random_vector()
        p2 = rnd_creator.generate_random_vector()
        p3 = rnd_creator.generate_random_vector()
        signed_area_sum = 0.
        vertices = [p1, p2, p3]
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % len(vertices)]
            signed_area_sum += (x1 * y2 - x2 * y1)
        tri = pycrcc.Triangle(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])

        if signed_area_sum < 0.:
            verts = tri.vertices()
            verts[1], verts[2] = verts[2], verts[1]
            if (verts != vertices):
                has_error = True
                print("triangle_polygon validation: error in created Triangle")
        else:
            verts = tri.vertices()
            if (verts != vertices):
                has_error = True
                print("triangle_polygon validation: error in created Triangle")
        poly = create_random_convex_polygon()
        verts_in = list(poly.exterior.coords)
        polygon = pycrcc.Polygon(verts_in, list(), list())
        verts = polygon.vertices()
        for ind, vert in enumerate(verts):
            verts[ind] = tuple(vert)
        if poly.exterior.is_ccw:
            if (verts != verts_in):
                has_error = True
        else:
            verts.reverse()
            if (verts != verts_in):
                has_error = True
    return has_error

if __name__ == "__main__":
    if run_test_triangle_polygon_reordering() == True:
        exit(1)