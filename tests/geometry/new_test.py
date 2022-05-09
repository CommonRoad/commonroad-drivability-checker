# commonroad
from commonroad.geometry.shape import Polygon, ShapeGroup, Shape, Rectangle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State

#commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader

# commonroad-boundary
from commonroad_dc.boundary import boundary
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle

from commonroad_dc.geometry import geometry
from commonroad_dc.geometry import util

import commonroad_dc.pycrcc as pycrcc
import commonroad_dc.pycrccosy as pycrccosy

from time import time
import numpy as np

import random
import math

viz_enable=False


import matplotlib.pyplot as plt

def open_scenario(scenario_filename):

    crfr = CommonRoadFileReader(
        scenario_filename)
    scenario, planning_problem_set = crfr.open()
    return scenario, planning_problem_set

import os

import pickle

import multiprocessing as mp
from multiprocessing.dummy import Pool as ThreadPool
from functools import partial

from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline


def compute_curvature_old(polyline):
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for curvature computation p={}'.format(polyline)

    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    # compute curvature
    curvature = (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))
    return curvature

def collectMyResult(result):
    print("Got result {}".format(str(result)))
    res_dict[result[0]]=(result[1],)


def scenario_time(path,id):

    time_start=time()

    sc_el = path.split("/")[-1]

    scenario, planning_problem_set = open_scenario(path)

    viz_enable=False

    lanelet_network = scenario.lanelet_network

    for lanelet in lanelet_network.lanelets:
        ref_path = lanelet.center_vertices
        #for i in range(0, 10):
        #ref_path = chaikins_corner_cutting(ref_path,10)
        #ref_path = resample_polyline(ref_path, 2.0)
        #if(len(ref_path<3)):
        #    continue
        try:
            res = geometry.CurvilinearCoordinateSystem(ref_path, 20., 0.1, 1e-4)
        except(geometry.RefPathLengthException):
            print("Reference path length is invalid")

            continue

        curvature1=compute_curvature_old(np.asarray(res.reference_path()))
        curvature2=util.compute_curvature_from_polyline(np.asarray(res.reference_path()))

        if not (np.isclose(curvature1,curvature2).all() and (len(curvature1)==len(curvature2))):
            print("Curvature computation error")

            continue

        import pickle
        cosy_dumps1 = pickle.dumps(res)
        obj2 = pickle.loads(cosy_dumps1)
        cosy_dumps2 = pickle.dumps(obj2)
        res.set_curvature(curvature2)
        cosy_dumps3 = pickle.dumps(res)
        obj2 = pickle.loads(cosy_dumps3)
        cosy_dumps4 = pickle.dumps(obj2)
        res.compute_and_set_curvature()
        cosy_dumps5 = pickle.dumps(res)
        obj2 = pickle.loads(cosy_dumps5)
        cosy_dumps6 = pickle.dumps(obj2)

        if not (cosy_dumps1==cosy_dumps2 and cosy_dumps3==cosy_dumps4 and cosy_dumps3==cosy_dumps5 and cosy_dumps3==cosy_dumps6):
            print("Curvature pickle error")

            continue

        for ind, x in enumerate(ref_path):
            try:
                res.convert_to_curvilinear_coords(x[0], x[1])
            except(Exception):
                pass

    # create curvilinear CoSy
    #curvilinear_cosy = pycrccosy.CurvilinearCoordinateSystem(ref_path, 25.0, 0.1, 1e-4)

    #if(id>134):
    #    viz_enable = True

    #if(viz_enable):
        # plot the scenario
    #    fig=plt.figure(figsize=(25, 10))
    #    draw_object(scenario)
    #    draw_object(planning_problem_set)
    #    plt.autoscale()
    #    plt.gca().set_aspect('equal')
    #    fig.savefig('plots/' + sc_el + '_scenario.png')
    #    plt.close(fig)

    #road_boundary_aa_triangulation=boundary.create_road_boundary_obstacle(scenario, method="aligned_triangulation", return_scenario_obstacle=False)
    #road_boundary_rectangles=boundary.create_road_boundary_obstacle(scenario, method="obb_rectangles", return_scenario_obstacle=False)

    #with open('pickle/'+sc_el+'_scenario.pickle','wb') as f:
    #    pickle.dump(scenario,f)

    #def save_boundary(road_boundary, method):
    #    # draw the road boundary
    #    fig = plt.figure(figsize=(25, 10))
    #    draw_object(road_boundary)
    #    plt.autoscale()
    #    plt.gca().set_aspect('equal')
    #    fig.savefig('plots/' + sc_el + '_boundary_' + method + '.png')
    #    with open('pickle/' + sc_el + '_boundary_' + method + '.pickle', 'wb') as f:
    #        pickle.dump(road_boundary, f)
    #    plt.close(fig)

    #if(viz_enable):
    #    save_boundary(road_boundary_aa_triangulation,'aligned_triangulation')
    #    save_boundary(road_boundary_rectangles,'obb_rectangles')

    #road_boundary_triangulation = boundary.create_road_boundary_obstacle(scenario, method="triangulation",     return_scenario_obstacle=False)


    #if (viz_enable):
    #    save_boundary(road_boundary_triangulation, 'triangulation')


    # length of the ego-vehicle
    #box_length=2.5
    # width of the ego-vehicle
    #box_width=1.05

    #list with trajectory states that are checked for collisions with the road boudnary
    #traj_list=list()

    return (path, time()-time_start)

def abortable_worker(func, *args, **kwargs):
    timeout = kwargs.get('timeout', None)
    p = ThreadPool(1)
    res = p.apply_async(scenario_time, args=args)
    try:
        out = res.get(timeout)  # Wait timeout seconds for func to complete.
        return out
    except mp.TimeoutError:
        print("Aborting due to timeout")
        p.terminate()
        return (args[0],'timeout', -1)

if(__name__=='__main__'):

    procs=list()

    res_dict=dict()

    dirname="/home/vit/Downloads/commonroad-scenarios-e31806e263a8a67937cfa7bf08ee1c8dc2a4c141/scenarios"
    paths=[os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(dirname)) for f in fn]
    print(len(paths))
    #paths = [x for x in paths if "ZAM_Intersect-1_1_S-1.xml" in x]
    paths=[x for x in paths if "ZAM_Urban-7_1_S-1.xml" not in x]
    paths = [x for x in paths if "ZAM_Urban-6_1_S-1.xml" not in x]
    paths = [x for x in paths if "DEU_Muc-17_1_T-1.xml" not in x]
    paths = [x for x in paths if "DEU_Gar-3_1_T-1.xml" not in x]
    paths = [x for x in paths if "DEU_Gar-3_2_T-1.xml" not in x]
    print(len(paths))
    #files=os.listdir(dirname)
    multiCore=True
    iter_max = 5000

    if(multiCore):

        pool = mp.Pool(8)
        tasksize= len(paths)
        for id,path in enumerate(paths):
            abortable_func = partial(abortable_worker, scenario_time, timeout=500)
            pool.apply_async(abortable_func, args=(path,id), callback=collectMyResult)
        pool.close()
        pool.join()

        with open('res_dict.txt', 'wb') as fout:
            pickle.dump(res_dict, fout)
    else:
        for i in range(iter_max):
            for id, path in enumerate(paths):
                print(scenario_time(path, id))
            print("cycle done \n")
