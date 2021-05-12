import sys
from typing import Tuple
import warnings

import commonroad.visualization.draw_dispatch_cr as draw_dispatch_core
import matplotlib.pyplot as plt

import commonroad_dc.collision.visualization.pycrcc


def _create_drawers_dict_pycrcc():
    draw_func_pycrcc = {}
    if 'commonroad_dc.collision.visualization.pycrcc' in sys.modules.keys():
        draw_func_pycrcc.update(commonroad_dc.collision.visualization.pycrcc.draw_func_dict)

    return draw_func_pycrcc


def _create_default_draw_params_pycrcc():
    draw_params = draw_dispatch_core._create_default_draw_params()
    if 'commonroad_dc.collision.visualization.pycrcc' in sys.modules.keys():
        draw_params.update(commonroad_dc.collision.visualization.pycrcc.create_default_draw_params())
    return draw_params


default_draw_params_pycrcc = _create_default_draw_params_pycrcc()


def _retrieve_value_by_path(style_sheet_caller: dict, value_path: Tuple[str, ...]):
    """
    Retrieves value corresponding to value path from the nested dict style_sheet.
    :param style_sheet_caller: parameters for plotting given by a nested dict
    :param value_path: string tuple that contains the path to the value
    :return: the value from style_sheet defined by value_path
    """

    c_dict = style_sheet_caller
    for value_element in value_path[:-1]:
        try:
            c_dict = c_dict[value_element]
        except KeyError:
            raise KeyError()
    try:
        return_value = c_dict[value_path[-1]]
    except KeyError:
        raise KeyError()
    return return_value


def _retrieve_value(style_sheet: dict, call_stack: tuple, value_path: Tuple[str, ...]):
    """
    Retrieves value corresponding to value_path from the nested dict style_sheet. If value_path not contained in
    style_sheet, try to retrieve from default draw_params
    Starts by searching for value_path beginning at first element in call stack, stepping down all frames
    :param style_sheet: parameters for plotting given by a nested dict
    :param call_stack: tuple of string containing the call stack
    :param value_path: string tuple that contains the path to the value
    :return: the value from style_sheet defined by value_path and call_stack
    """

    # first: try to retrieve value with call stack, start from deepest level
    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    # try to retrieve value without call_stack (for fallback to top-level parameters)
    try:
        value = _retrieve_value_by_path(style_sheet, value_path)
        return value
    except KeyError:
        pass

    # try to retrieve from default parameters

    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = default_draw_params_pycrcc
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    try:
        value = _retrieve_value_by_path(default_draw_params_pycrcc, value_path)
        return value
    except KeyError:
        pass

    # if value not found yet, it is neither given in default parameters nor in provided parameters,
    # possibly a wrong call_stack was provided
    try:
        value
    except NameError:
        raise KeyError
    return value


def _retrieve_alternate_value(style_sheet: dict, call_stack: Tuple[str, ...], value_path_1: Tuple[str, ...],
                              value_path_2: Tuple[str, ...]):
    """
    Like retrieve_value(...), but retrieves value from value_path_2 if value_path_1 does not exist in style_sheet
    :param style_sheet: parameters for plotting given by a nested dict (see draw_params in draw_object)
    :param call_stack: tuple of string containing the call stack
    :param value_path_1: string tuple that contains the path to the value
    :param value_path_2: alternate value_path
    :return: the value from style_sheet defined by value_path_1 (or value_path_2)
    """
    # first: try to retrieve value with call stack, start from deepest level
    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path_1)
                return value
            except KeyError:
                pass
        except KeyError:
            continue

    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path_2)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    # try to retrieve value without call_stack (for fallback to top-level parameters)
    try:
        value = _retrieve_value_by_path(style_sheet, value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(style_sheet, value_path_2)
        return value
    except KeyError:
        pass

    # try to retrieve from default parameters
    try:
        value = _retrieve_value_by_path(default_draw_params_pycrcc, call_stack + value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params_pycrcc, call_stack + value_path_2)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params_pycrcc, value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params_pycrcc, value_path_2)
        return value
    except KeyError:
        pass

    # if value not found yet, it is neither given in default parameters nor in provided parameters,
    # possibly a wrong call_stack was provided
    try:
        value
    except NameError:
        raise KeyError
    return value


def draw_object(obj, plot_limits=None, ax=None, draw_params=None,
                draw_func=None, handles=None, call_stack=None):
    """
    Main function for drawing objects from the commonroad-collision-checker.

    :param obj: the object or list of objects (with all the same type) to be plotted
    :param plot_limits:  list of [x_min, x_max, y_min, y_max] that defines the plotted area
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
           see documentation for full overview over the structure. If parameters are not set,
           the default setting are used.
    :param draw_func: specify the drawing function (usually not necessary to change default)
    :param handles: dict that assign to every object_id of all plotted obstacles the corresponding patch handles
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object, (usually 'None'!)
    :return: Returns matplotlib patch object for draw_funcs that actually draw a patch (used internally for creating handles dict)
    """
    warnings.warn('draw_object is deprecated, use renderer interface instead',
                  DeprecationWarning)
    if handles is None:
        handles = dict()

    if ax is None:
        ax = plt.gca()

    if plot_limits is not None:
        ax.set_xlim(plot_limits[0], plot_limits[1])
        ax.set_ylim(plot_limits[2], plot_limits[3])

    if draw_func is None:
        draw_func_pycrcc = _create_drawers_dict_pycrcc()
    else:
        draw_func_pycrcc = draw_func

    if draw_params is None:
        draw_params = _create_default_draw_params_pycrcc()

    if call_stack is None:
        call_stack = tuple()

    if type(obj) is list:
        if len(obj) == 0:
            return []
        for o in obj:
            draw_object(o, None, ax, draw_params, draw_func, handles, call_stack)
        return

    draw_func_core = draw_dispatch_core._create_drawers_dict()

    if type(obj) in draw_func_pycrcc.keys():
        draw_func_pycrcc[type(obj)](obj, plot_limits, ax, draw_params, draw_func_pycrcc, handles, call_stack)
    elif type(obj) in draw_func_core.keys():
        draw_dispatch_core.draw_object(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack)
    else:
        print("Cannot dispatch to plot " + str(type(obj)))
