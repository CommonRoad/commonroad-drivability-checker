import commonroad.geometry.occupancy.occupancy
import commonroad.geometry.occupancy.rect_occupancy
import commonroad.geometry.occupancy.circle_occupancy
import commonroad.geometry.occupancy.polygon_occupancy
import commonroad.geometry.occupancy.occupancy_group

import numpy as np
import shapely


def minkowski_sum_circle(shape: commonroad.geometry.occupancy.occupancy.Occupancy,
                         radius: float, resolution: int) -> commonroad.geometry.occupancy.occupancy.Occupancy:
    return minkowski_sum_circle_func_dict[type(shape)](shape, radius, resolution)


def minkowski_sum_circle_shapely_polygon(polygon: shapely.geometry.Polygon,
                                         radius: float, resolution: int) -> np.ndarray:
    """
    Computes the minkowski sum of a provided polygon and a circle with
    parametrized radius

    :param polygon: The polygon as a numpy array with columns as x and y
    coordinates
    :param radius: The radius of the circle
    :return: The minkowski sum of the provided polygon and the circle with the
    parametrized radius
    """
    assert isinstance(polygon, shapely.geometry.Polygon), \
        '<> Provided polygon is not an instance of shapely.geometry.Polygon,' \
        ' polygon = {}'.format(polygon)
    assert radius > 0, '<> Provided radius must be positive. radius = {}'. \
        format(radius)
    # dilation with round cap style (style = 1)
    dilation = polygon.buffer(radius, cap_style=1, resolution=resolution)
    if isinstance(dilation, shapely.geometry.MultiPolygon):
        polys = [polygon for polygon in dilation]
        for p in polys:
            p_vertices = list()
            vertices = p.exterior.coords
            for v in vertices:
                p_vertices.append(np.array([v[0], v[1]]))
    # convert to array
    else:
        p_vertices = list()
        vertices = dilation.exterior.coords
        for v in vertices:
            p_vertices.append(np.array([v[0], v[1]]))
    return np.array(p_vertices)


def minkowski_sum_circle_polygon(polygon: commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy,
                                 radius: float, resolution: int) \
        -> commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy:
    if np.isclose(radius, 0.0):
        return polygon
    else:
        return commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy(
            polygon = shapely.geometry.Polygon(minkowski_sum_circle_shapely_polygon(polygon.shapely_object, radius, resolution)))


def minkowski_sum_circle_circle(circle:  commonroad.geometry.occupancy.circle_occupancy.CircleOccupancy,
                                radius: float, resolution: int) \
        ->  commonroad.geometry.occupancy.circle_occupancy.CircleOccupancy:
    return  commonroad.geometry.occupancy.circle_occupancy.CircleOccupancy(
        radius = circle.radius + radius, circle_center = circle.circle_center)


def minkowski_sum_circle_rectangle(
        rectangle: commonroad.geometry.occupancy.rect_occupancy.RectOccupancy, radius: float, resolution: int) \
        -> commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy:
    return commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy(
        polygon = shapely.geometry.Polygon(minkowski_sum_circle_shapely_polygon(rectangle.shapely_object, radius, resolution)))


def minkowski_sum_circle_occupancy_group(
        occupancy_group: commonroad.geometry.occupancy.occupancy_group.OccupancyGroup, radius: float, resolution: int) \
        -> commonroad.geometry.occupancy.occupancy_group.OccupancyGroup:
    new_occupancies = list()
    for s in occupancy_group.occupancies:
        new_occupancies.append(minkowski_sum_circle(s, radius, resolution))
    return commonroad.geometry.occupancy.occupancy_group.OccupancyGroup(occupancies = tuple(new_occupancies))


minkowski_sum_circle_func_dict = {
    commonroad.geometry.occupancy.occupancy_group.OccupancyGroup: minkowski_sum_circle_occupancy_group,
    commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy: minkowski_sum_circle_polygon,
    commonroad.geometry.occupancy.circle_occupancy.CircleOccupancy: minkowski_sum_circle_circle,
    commonroad.geometry.occupancy.rect_occupancy.RectOccupancy: minkowski_sum_circle_rectangle,
}
