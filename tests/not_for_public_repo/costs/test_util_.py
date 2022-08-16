import unittest
import os
import matplotlib
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad_dc import pycrccosy

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc.costs.route_matcher import *

draw = True


class TestLaneletNetworkMatcher(unittest.TestCase):
    def setUp(self) -> None:
        self.test_ressources_dir = os.path.join(os.path.dirname(__file__), "ressources")
        self.test_scenario_dir = os.path.join(self.test_ressources_dir, "scenarios")

    def test_find_lanelet_by_trajectory(self):
        scenario_name = "USA_Lanker-2_23_T-1.xml"
        scenario, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        lnm = LaneletRouteMatcher(scenario, vehicle_type=VehicleType.FORD_ESCORT)
        for obs in scenario.dynamic_obstacles[10:20]:
            lanelets, props = lnm.find_lanelets_by_trajectory(obs.prediction.trajectory,
                                                       required_properties=[SolutionProperties.LonJerk])

            if draw is True:
                l_tmp = LaneletNetwork.create_from_lanelet_list([lnm.lanelet_network._lanelets[l] for l in lanelets])
                plt.figure()
                rnd = MPRenderer()
                lnm.lanelet_network.draw(rnd)
                l_tmp.draw(draw_params={'lanelet': {'facecolor': 'red'},   "traffic_sign": {
                    "draw_traffic_signs": False},  "traffic_light": {
                    "draw_traffic_lights": False}}, renderer=rnd)
                obs.draw(draw_params={'time_begin':0, 'time_end': obs.prediction.final_time_step}, renderer=rnd)
                rnd.render(show=False)
                plt.show(block=False)

                plt.pause(0.01)

        if draw is True:
            plt.pause(1000)

    def test_find_lanelet_by_trajectory_oncoming_in_intersection(self):
        """Test whether correct path is found in intersection where vehicle drives into oncoming lane
        (no adjacency relation given)"""
        scenario_name = "DEU_Ibbenbueren-10_2_T-1.xml"
        solution_name = "solution_KS2:SM1:DEU_Ibbenbueren-10_2_T-1:2020a.xml"
        scenario, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        solution = CommonRoadSolutionReader.open(os.path.join(self.test_scenario_dir, solution_name))
        lnm = LaneletRouteMatcher(scenario, vehicle_type=VehicleType.FORD_ESCORT)
        lanelets, props = lnm.find_lanelets_by_trajectory(solution.planning_problem_solutions[0].trajectory,
                                                   required_properties=[SolutionProperties.LonJerk])

        self.assertEqual([31740, 36042], lanelets)

        if draw is True:
            l_tmp = LaneletNetwork.create_from_lanelet_list([lnm.lanelet_network._lanelets[l] for l in lanelets])
            plt.figure()
            rnd = MPRenderer()
            lnm.lanelet_network.draw(rnd)
            l_tmp.draw(draw_params={'lanelet': {'facecolor': 'red'},   "traffic_sign": {
                "draw_traffic_signs": False},  "traffic_light": {
                "draw_traffic_lights": False}}, renderer=rnd)
            solution.planning_problem_solutions[0].trajectory.draw(draw_params={'time_begin':0}, renderer=rnd)
            rnd.render(show=False)
            plt.show(block=False)

            plt.pause(1000)

    def test_compute_curvilinear_coordinates(self):
        cy = pycrccosy.CurvilinearCoordinateSystem([[1.0, 1.0], [2.0, 1.0], [3.0, 1.0], [4.0, 1.0]])

        scenario_name = "USA_US101-23_1_T-1.xml"
        scenario, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        lnm = LaneletRouteMatcher(scenario, vehicle_type=VehicleType.FORD_ESCORT)
        for obs in scenario.dynamic_obstacles:
            if obs.obstacle_id not in [90,74]:
                continue
            trajectory, lanelets, props = lnm.compute_curvilinear_coordinates(obs.prediction.trajectory,
                                                       required_properties=[SolutionProperties.LonJerk])

            if draw is True:
                l_tmp = LaneletNetwork.create_from_lanelet_list([lnm.lanelet_network._lanelets[l] for l in lanelets])
                plt.figure()
                rnd = MPRenderer()
                lnm.lanelet_network.draw(rnd)
                l_tmp.draw(draw_params={'lanelet': {'facecolor': 'red'},   "traffic_sign": {
        "draw_traffic_signs": False},  "traffic_light": {
        "draw_traffic_lights": False}}, renderer=rnd)
                obs.draw(draw_params={'time_begin':0, 'time_end': obs.prediction.final_time_step}, renderer=rnd)
                rnd.render(show=False)
                plt.show(block=False)

                plt.figure()
                plt.plot([s.lon_jerk for s in trajectory.state_list])
                plt.autoscale()
                plt.show(block=True)
                plt.pause(0.01)

        if draw is True:
            plt.pause(1000)


    def test_curvilinear_state(self):
        cs = CurvilinearState(velocity=1.0, lon_jerk=0)
        self.assertTrue(cs.velocity == 1.0)
        self.assertTrue(cs.lon_jerk == 0)



if __name__ == '__main__':
    unittest.main()
