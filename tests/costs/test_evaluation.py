import random
import warnings

import matplotlib
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.feasibility.feasibility_checker import input_vector_feasibility
from commonroad_dc.feasibility.solution_checker import valid_solution
from commonroad_dc.feasibility.vehicle_dynamics import PointMassDynamics

matplotlib.use('TkAgg')
import glob
import os
import unittest
from copy import deepcopy

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader, CostFunction, VehicleType
from commonroad.scenario.scenario import ScenarioID
from commonroad_dc.costs.evaluation import CostFunctionEvaluator, PartialCostFunction


class TestCostFunctionEvaluator(unittest.TestCase):
    def setUp(self) -> None:
        self.test_ressources_dir = os.path.join(os.path.dirname(__file__), "ressources")
        self.test_scenario_dir = os.path.join(self.test_ressources_dir, "scenarios")
        self.test_solutions_dir = os.path.join(self.test_ressources_dir, "example_solutions")

        self.all_scenario_dir = "/home/klischat/GIT_REPOS/commonroad-scenarios-dev/scenarios"
        self.cost_funcs = [CostFunction.JB1,
                           CostFunction.MW1,
                           CostFunction.SA1,
                           CostFunction.WX1,
                           CostFunction.SM1,
                           CostFunction.SM2,
                           CostFunction.SM3]

    def _get_scenario_paths(self, scenario_id: ScenarioID = None):
        if scenario_id is None:
            return glob.glob(os.path.join(self.all_scenario_dir, "*.xml"), recursive=True)
        else:
            return glob.glob(os.path.join(self.all_scenario_dir, f"**/*/{str(scenario_id)}.xml"), recursive=True)

    def _open_scenario_by_id(self, scenario_id: ScenarioID):
        paths = self._get_scenario_paths(scenario_id)
        if not paths:
            raise ValueError(f"No scenario with ID {scenario_id} found in path {self.all_scenario_dir}")
        scenario, pp = CommonRoadFileReader(paths[0]).open()
        if not scenario.scenario_id == scenario_id:
            warnings.warn(
                f"{str(scenario.scenario_id)}:{scenario.scenario_id.scenario_version} "
                f" != {str(scenario_id)}:{scenario_id.scenario_version}")
        return scenario, pp

    def _open_solution_scenario(self, solution_path: str):
        solution = CommonRoadSolutionReader.open(solution_path)
        s, pp = self._open_scenario_by_id(solution.scenario_id)
        return solution, s, pp

    def test_init(self):

        for c in self.cost_funcs:
            ce = CostFunctionEvaluator(c, vehicle_type=VehicleType.FORD_ESCORT)
            rp = ce.required_properties

    def test_costs(self):
        scenario_name = "USA_Lanker-2_23_T-1.xml"
        scenario, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        cost_funcs = [
            CostFunction.JB1,
            CostFunction.MW1,
            CostFunction.SA1,
            CostFunction.WX1,
            CostFunction.SM1,
            CostFunction.SM2,
            CostFunction.SM3
        ]
        for c in cost_funcs:
            # for vehicle_type in VehicleType.__members__:
            vehicle_type = "FORD_ESCORT"
            ce = CostFunctionEvaluator(c, vehicle_type=VehicleType[vehicle_type])
            for obs in scenario.dynamic_obstacles[:10]:
                # if not obs.obstacle_id == 21241:
                #     continue
                print(obs.obstacle_id)

                scenario_copy = deepcopy(scenario)
                scenario_copy.remove_obstacle(obs)
                traj_tmp = deepcopy(obs.prediction.trajectory)
                for s in traj_tmp.state_list:
                    s.steering_angle = random.uniform(-0.05, 0.05)

                ce.evaluate_pp_solution(scenario, list(pp.planning_problem_dict.values())[0],
                                        traj_tmp)

    def test_solutions(self):
        """
        Tests 100+ solutions and visualizes macthed routes for inspection
        :return:
        """
        vt = VehicleType.FORD_ESCORT
        cost_funcs = [
            # CostFunction.JB1,
            CostFunction.MW1,
            # CostFunction.SA1,
            # CostFunction.WX1,
            # CostFunction.SM1,
            # CostFunction.SM2,
            # CostFunction.SM3
        ]
        g0 = False
        for s in list(glob.glob(os.path.join(self.test_solutions_dir, "*.xml"), recursive=True))[:]:
            # if not "Lohmar-23_1" in s:
            #     continue
            # if not "Moabit-6_2" in s:
            #     continue
            # SIGSEV error:
            # if not "USA_US101-8_4_T" in s:
            #     continue
            # with open('/home/klischat/Downloads/2101_tmp_videos/test.txt', 'w') as f:
            #     print(str(s), file=f)
            # Abort (init border not in domain)
            # if not "USA_US101-8_4_T-1" in s:
            #     continue
            if not "DEU_Flensburg-2_1_T-1" in s and not g0:
                continue
            g0=True

            sol, sce, pp = self._open_solution_scenario(s)
            print(sce.scenario_id)
            for c_fun in cost_funcs:
                for pp_sol in sol.planning_problem_solutions:
                    pp_sol.cost_function = c_fun
                    ce = CostFunctionEvaluator(c_fun, vt)
                    if pp_sol.planning_problem_id not in pp.planning_problem_dict:
                        continue
                    ce.evaluate_pp_solution(cr_scenario=sce,
                                            cr_pproblem=pp.planning_problem_dict[pp_sol.planning_problem_id],
                                            trajectory=pp_sol.trajectory, draw_lanelet_path=True)

    def test_pm_input_solution(self):
        cost_funcs = [
            # CostFunction.JB1,
            CostFunction.MW1,
            # CostFunction.SA1,
            # CostFunction.WX1,
            # CostFunction.SM1,
            # CostFunction.SM2,
            # CostFunction.SM3
        ]
        scenario_name = "ZAM_Tjunction-1_18_T-1.xml"
        solution_file = "solution_PMInput.xml"
        sol = CommonRoadSolutionReader.open(os.path.join(self.test_solutions_dir, solution_file))
        sce, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        feasible, results = valid_solution(sce, pp, sol)
        for c_fun in cost_funcs:
            for pp_sol in sol.planning_problem_solutions:
                print(results[pp_sol.planning_problem_id][2].state_list[0])

                pp_sol.cost_function = c_fun
                ce = CostFunctionEvaluator(c_fun, pp_sol.vehicle_type)
                ce.evaluate_pp_solution(cr_scenario=sce,
                                        cr_pproblem=pp.planning_problem_dict[pp_sol.planning_problem_id],
                                        trajectory=results[pp_sol.planning_problem_id][2])

    def test_vis(self):
        solution_file = "solution_PMInput.xml"
        scenario_name = "ZAM_Tjunction-1_18_T-1.xml"
        sce, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        sol = CommonRoadSolutionReader.open(os.path.join(self.test_solutions_dir, solution_file))
        feas, traj = input_vector_feasibility(pp.planning_problem_dict[sol.planning_problem_solutions[0].planning_problem_id].initial_state,
                                 list(sol.planning_problem_solutions)[0].trajectory, PointMassDynamics(VehicleType.BMW_320i),sce.dt)
        rnd = MPRenderer()
        rnd.draw_trajectory(traj,draw_params=rnd.draw_params,
                            call_stack=())
        rnd.render()

    def test_pm_input_solution_reconstruction(self):
        cost_funcs = [
            CostFunction.JB1,
            # CostFunction.MW1,
            # CostFunction.SA1,
            # CostFunction.WX1,
            # CostFunction.SM1,
            # CostFunction.SM2,
            # CostFunction.SM3
        ]
        scenario_name = "ZAM_Tjunction-1_18_T-1.xml"
        solution_file = "solution_PMInput.xml"
        sol = CommonRoadSolutionReader.open(os.path.join(self.test_solutions_dir, solution_file))
        sce, pp = CommonRoadFileReader(os.path.join(self.test_scenario_dir, scenario_name)).open()
        feasible, results = valid_solution(sce, pp, sol)
        import matplotlib.pyplot as plt
        import numpy as np
        for k, v in results.items():
            plt.figure()
            acc = np.array([[s.velocity, s.velocity_y] for s in v[2].state_list])
            plt.plot(acc[:30, 0], acc[:30, 1])
            plt.show(block=False)

            plt.figure()
            acc = np.array([[s.acceleration, s.acceleration_y] for s in v[1].state_list])
            plt.plot(acc[:30, 0], acc[:30, 1])
            plt.show(block=False)

            plt.figure()
            p = np.array([s.position for s in v[2].state_list])
            plt.plot(p[:,0], p[:,1])
            plt.show(block=True)


if __name__ == '__main__':
    unittest.main()
    # tt = TestCostFunctionEvaluator()
    # tt.setUp()
    # tt.test_pm_input_solution()