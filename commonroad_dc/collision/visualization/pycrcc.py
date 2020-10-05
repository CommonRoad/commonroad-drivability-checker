import matplotlib.colors
import matplotlib.patches
import commonroad_dc.pycrcc as pycrcc
from commonroad.visualization.util import draw_polygon_as_patch

import commonroad_dc.collision


def create_default_draw_params():
    draw_params = {'rectobb': {'facecolor': '#ff9999',
                               'edgecolor': '#000000',
                               'opacity': 1,
                               'zorder': 20},
                   'rectaabb': {'facecolor': '#ff9999',
                                'edgecolor': '#000000',
                                'opacity': 1,
                                'zorder': 20},
                   'triangle': {'facecolor': '#ff9999',
                                'edgecolor': '#000000',
                                'opacity': 1,
                                'zorder': 20},
                   'circle': {'facecolor': '#ff9999',
                              'edgecolor': '#000000',
                              'opacity': 1,
                              'zorder': 20},
                   'point': {'facecolor': '#ff9999',
                             'opacity': 1,
                             'zorder': 20},
                   'polygon': {'facecolor': '#ff9999',
                               'edgecolor': '#000000',
                               'opacity': 1,
                               'zorder': 20,
                               'draw_mesh': False}}
    draw_params = {'collision': draw_params}
    draw_params['rectobb'] = draw_params['collision']['rectobb']
    draw_params['rectaabb'] = draw_params['collision']['rectaabb']

    return draw_params


def draw_collision_point(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'point', 'facecolor'),
            ('collision', 'facecolor'))
        opacity = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'point', 'opacity'),
            ('collision', 'opacity'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'point', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for point. Called through:")
        print(call_stack)
    ax.plot(obj.center()[0], obj.center()[1], alpha=opacity, color=facecolor, zorder=zorder)


def draw_collision_rectaabb(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'edgecolor'),
            ('collision', 'edgecolor'))
        opacity = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'opacity'),
            ('collision', 'opacity'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectaabb', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for rectaabb. Called through:")
        print(call_stack)

    vertices = [(obj.min_x(), obj.min_y()),
                (obj.min_x(), obj.max_y()),
                (obj.max_x(), obj.max_y()),
                (obj.max_x(), obj.min_y())]

    draw_polygon_as_patch(vertices, ax, zorder=zorder, facecolor=facecolor,
                          edgecolor=edgecolor, lw=0.5, alpha=opacity)


def draw_collision_rectobb(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'rectobb', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'rectobb', 'edgecolor'),
            ('collision', 'edgecolor'))
        opacity = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectobb', 'opacity'),
            ('collision', 'opacity'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'rectobb', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for rectobb. Called through:")
        print(call_stack)
    center = obj.center()
    r_x = obj.r_x()
    r_y = obj.r_y()
    a_x = obj.local_x_axis()
    a_y = obj.local_y_axis()
    pt1 = center + r_x * a_x + r_y * a_y
    pt2 = center - r_x * a_x + r_y * a_y
    pt3 = center - r_x * a_x - r_y * a_y
    pt4 = center + r_x * a_x - r_y * a_y

    vertices = [(pt1[0], pt1[1]),
                (pt2[0], pt2[1]),
                (pt3[0], pt3[1]),
                (pt4[0], pt4[1])]

    draw_polygon_as_patch(vertices, ax,
                          zorder=zorder,
                          facecolor=facecolor,
                          edgecolor=edgecolor,
                          alpha=opacity,
                          lw=0.5)


def draw_collision_triangle(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'triangle', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'triangle', 'edgecolor'),
            ('collision', 'edgecolor'))
        opacity = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'triangle', 'opacity'),
            ('collision', 'opacity'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'triangle', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for triangle. Called through:")
        print(call_stack)
    v = obj.vertices()
    vertices = [(v[0][0], v[0][1]),
                (v[1][0], v[1][1]),
                (v[2][0], v[2][1])]

    draw_polygon_as_patch(vertices, ax, zorder=zorder, facecolor=facecolor, alpha=opacity,
                          edgecolor=edgecolor, lw=0.5)


def draw_collision_circle(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'circle', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack, ('collision', 'circle', 'edgecolor'),
            ('collision', 'edgecolor'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'circle', 'zorder'),
            ('collision', 'zorder'))
    except KeyError:
        print("Cannot find stylesheet for circle. Called through:")
        print(call_stack)

    patch = matplotlib.patches.Ellipse([obj.x(), obj.y()], 2 * obj.r(),
                                       2 * obj.r(), zorder=zorder,
                                       facecolor=facecolor,
                                       edgecolor=edgecolor, lw=0.5)
    ax.add_patch(patch)


def draw_collision_timevariantcollisionobject(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    call_stack = tuple(list(call_stack) + ['time_variant_obstacle'])
    for i in range(obj.time_start_idx(), obj.time_end_idx() + 1):
        tmp = obj.obstacle_at_time(i)
        if "time_to_color" in draw_params.keys():
            draw_params = draw_params.copy()
            draw_params['color'] = draw_params["time_to_color"].to_rgba(i)
            commonroad_dc.collision.visualization.draw_dispatch.draw_object(obj, plot_limits, ax, draw_params,
                                                                            draw_func, handles, call_stack)
        else:
            commonroad_dc.collision.visualization.draw_dispatch.draw_object(tmp, plot_limits, ax, draw_params,
                                                                            draw_func, handles, call_stack)


def draw_collision_shapegroup(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    call_stack = tuple(list(call_stack) + ['shape_group'])
    for o in obj.unpack():
        commonroad_dc.collision.visualization.draw_dispatch.draw_object(o, plot_limits, ax, draw_params,
                                                                        draw_func, handles, call_stack)


def draw_collision_polygon(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    try:
        facecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'facecolor'),
            ('collision', 'facecolor'))
        edgecolor = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'edgecolor'),
            ('collision', 'edgecolor'))
        opacity = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'opacity'),
            ('collision', 'opacity'))
        zorder = commonroad_dc.collision.visualization.draw_dispatch._retrieve_alternate_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'zorder'),
            ('collision', 'zorder'))
        draw_mesh = commonroad_dc.collision.visualization.draw_dispatch._retrieve_value(
            draw_params, call_stack,
            ('collision', 'polygon', 'draw_mesh'))
    except KeyError:
        print("Cannot find stylesheet for polygon. Called through:")
        print(call_stack)
    call_stack = tuple(list(call_stack) + ['polygon'])
    if draw_mesh:
        for o in obj.triangle_mesh():
            commonroad_dc.collision.visualization.draw_dispatch.draw_object(o, plot_limits, ax, draw_params,
                                                                            draw_func, handles, call_stack)
    else:
        draw_polygon_as_patch(obj.vertices(), ax, zorder=zorder, facecolor=facecolor,
                              edgecolor=edgecolor, lw=0.5, alpha=opacity)


def draw_collision_collisionchecker(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack):
    call_stack = tuple(list(call_stack) + ['collision_checker'])
    for o in obj.obstacles():
        commonroad_dc.collision.visualization.draw_dispatch.draw_object(o, plot_limits, ax, draw_params,
                                                                        draw_func, handles, call_stack)


draw_func_dict = {pycrcc.RectAABB: draw_collision_rectaabb,
                  pycrcc.RectOBB: draw_collision_rectobb,
                  pycrcc.Circle: draw_collision_circle,
                  pycrcc.Triangle: draw_collision_triangle,
                  pycrcc.TimeVariantCollisionObject:
                      draw_collision_timevariantcollisionobject,
                  pycrcc.ShapeGroup: draw_collision_shapegroup,
                  pycrcc.Polygon: draw_collision_polygon,
                  pycrcc.CollisionChecker:
                      draw_collision_collisionchecker,
                  pycrcc.Point: draw_collision_point}
