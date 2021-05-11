# commonroad
from commonroad.geometry.shape import Polygon, ShapeGroup, Shape, Rectangle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State

#commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc.collision.visualization.draw_dispatch import draw_object

# commonroad-boundary
from commonroad_dc.boundary import boundary
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle

import commonroad_dc.pycrcc as pycrcc

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

def collectMyResult(result):
    print("Got result {}".format(str(result)))
    res_dict[result[0]]=(result[1],)


def scenario_time(path,id):

    time_start=time()

    sc_el = path.split("/")[-1]

    scenario, planning_problem_set = open_scenario(path)

    viz_enable=True

    #if(id>134):
    #    viz_enable = True

    if(viz_enable):
        # plot the scenario
        fig=plt.figure(figsize=(25, 10))
        draw_object(scenario)
        draw_object(planning_problem_set)
        plt.autoscale()
        plt.gca().set_aspect('equal')
        fig.savefig('plots/' + sc_el + '_scenario.png')
        plt.close(fig)

    road_boundary_aa_triangulation=boundary.create_road_boundary_obstacle(scenario, method="aligned_triangulation", return_scenario_obstacle=False)
    road_boundary_rectangles=boundary.create_road_boundary_obstacle(scenario, method="obb_rectangles", return_scenario_obstacle=False)

    with open('pickle/'+sc_el+'_scenario.pickle','wb') as f:
        pickle.dump(scenario,f)

    def save_boundary(road_boundary, method):
        # draw the road boundary
        fig = plt.figure(figsize=(25, 10))
        draw_object(road_boundary)
        plt.autoscale()
        plt.gca().set_aspect('equal')
        fig.savefig('plots/' + sc_el + '_boundary_' + method + '.png')
        with open('pickle/' + sc_el + '_boundary_' + method + '.pickle', 'wb') as f:
            pickle.dump(road_boundary, f)
        plt.close(fig)

    if(viz_enable):
        save_boundary(road_boundary_aa_triangulation,'aligned_triangulation')
        save_boundary(road_boundary_rectangles,'obb_rectangles')

    road_boundary_triangulation = boundary.create_road_boundary_obstacle(scenario, method="triangulation",     return_scenario_obstacle=False)


    if (viz_enable):
        save_boundary(road_boundary_triangulation, 'triangulation')


    # length of the ego-vehicle
    box_length=2.5
    # width of the ego-vehicle
    box_width=1.05

    #list with trajectory states that are checked for collisions with the road boudnary
    traj_list=list()

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

    dirname="[DIRECTORY]/commonroad-scenarios/scenarios"
    paths=[os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(dirname)) for f in fn]
    print(len(paths))
    paths=[x for x in paths if "ZAM_Urban-7_1_S-1.xml" not in x]
    paths = [x for x in paths if "ZAM_Urban-6_1_S-1.xml" not in x]
    paths = [x for x in paths if "DEU_Muc-17_1_T-1.xml" not in x]
    paths = [x for x in paths if "DEU_Gar-3_1_T-1.xml" not in x]
    paths = [x for x in paths if "DEU_Gar-3_2_T-1.xml" not in x]
    print(len(paths))
    #files=os.listdir(dirname)
    multiCore=True

    if(multiCore):


        pool = mp.Pool(8)

        tasksize= len(paths)
        for id,path in enumerate(paths):
            abortable_func = partial(abortable_worker, scenario_time, timeout=500)
            pool.apply_async(abortable_func, args=(path,id), callback=collectMyResult)
        pool.close()
        pool.join()

        with open('res_dict.txt','wb') as fout:
            pickle.dump(res_dict,fout)
    else:
        while(True):
            for id,path in enumerate(paths):
                print(scenario_time(path,id))
            print("cycle done \n")

