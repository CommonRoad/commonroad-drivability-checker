import glob
import os.path
import time
import unittest
import copy

from test_costs_base import TestSolutionEvaluationBase

import cpp_env_model
import commonroad_monitor
from commonroad.common.file_reader import CommonRoadFileReader


class TrafficRuleTests(TestSolutionEvaluationBase):
    def setUp(self) -> None:
        super().setUp()
        self.rule_folder = os.path.join(os.path.dirname(__file__), "ressources/traffic_rules/rules/")
        self.scenario_folder = os.path.join(os.path.dirname(__file__), "ressources/traffic_rules/scenarios/")

    # def test_basic_functionality(self):
    #     commonroad_monitor.register_monitor(1, os.path.join(self.rule_folder, "traffic_rules.yaml"))
    #     scenario, _ = CommonRoadFileReader(os.path.join(self.scenario_folder, "DEU_test_safe_distance.xml")).open()
    #     activated_traffic_rule_sets = ["SRG1"]
    #     for idx in range(len(scenario.dynamic_obstacles)):
    #         t0 = time.time()
    #         obstacles = copy.deepcopy(scenario.dynamic_obstacles)
    #         ego_vehicle = obstacles[idx]
    #         obstacles.remove(ego_vehicle)
    #         cpp_env_model.register_scenario(123, 0, "DEU", scenario.lanelet_network, obstacles, [ego_vehicle])
    #         print(commonroad_monitor.evaluate_rules(123, ego_vehicle.obstacle_id, activated_traffic_rule_sets))
    #         cpp_env_model.remove_scenario(123)
    #         print(time.time() -t0)

    def test_tr_with_solutions(self):
        def timeout(func, args=(), kwargs={}, timeout_duration=1, default=None):
            import signal

            class TimeoutError(Exception):
                pass

            def handler(signum, frame):
                raise TimeoutError()

            # set the timeout handler
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(timeout_duration)
            try:
                result = func(*args, **kwargs)
            except TimeoutError as exc:
                result = default
            finally:
                signal.alarm(0)

            return result

        commonroad_monitor.register_monitor(1, os.path.join(self.rule_folder, "traffic_rules.yaml"))
        activated_traffic_rule_sets = ["SRG1"]
        success = 0
        n_sol = 100
        for i_s, s in enumerate(list(glob.glob(os.path.join(self.test_solutions_dir, "*.xml"), recursive=True))[:n_sol]):
            sol, scenario, pp = self._open_solution_scenario(s)
            ego_obs = list(sol.create_dynamic_obstacle().values())
            print(str(scenario.scenario_id))
            # scenario, _ = CommonRoadFileReader(os.path.join(self.scenario_folder, "DEU_test_safe_distance.xml")).open()
            try:
            # timeout(cpp_env_model.register_scenario,(i_s, 0,
                                            # scenario.scenario_id.country_id,
                                            # scenario.lanelet_network,
                                            # scenario.obstacles,
                                            # ego_obs[:1]), timeout_duration=1)
                cpp_env_model.register_scenario(i_s, 0,
                                                scenario.scenario_id.country_id,
                                                scenario.lanelet_network,
                                                scenario.obstacles,
                                                ego_obs[:1])

                for e in ego_obs:
                    print(commonroad_monitor.evaluate_rules(i_s, e.obstacle_id, activated_traffic_rule_sets))
                cpp_env_model.remove_scenario(i_s)
                success += 1
            except BaseException as e:
                print(e)
                continue

        print(f"solved {success} solutions out of {len(n_sol)}")

    def test_tr_registration(self):
        commonroad_monitor.register_monitor(1, os.path.join(self.rule_folder, "traffic_rules.yaml"))
        activated_traffic_rule_sets = ["SRG1"]
        for i_s, s in enumerate(list(glob.glob(os.path.join("/home/klischat/GIT_REPOS/commonroad-scenarios-dev/scenarios/scenario-factory", "**/*.xml"), recursive=True))[:50]):
            scenario, pp = CommonRoadFileReader(s).open()
            print(str(scenario.scenario_id))
            # if len(scenario.lanelet_network.intersections) > 0:
            #     continue
            # scenario, _ = CommonRoadFileReader(os.path.join(self.scenario_folder, "DEU_test_safe_distance.xml")).open()
            print(i_s)
            try:
                # timeout(cpp_env_model.register_scenario,(i_s, 0,
                # scenario.scenario_id.country_id,
                # scenario.lanelet_network,
                # scenario.obstacles,
                # ego_obs[:1]), timeout_duration=1)
                cpp_env_model.register_scenario(i_s, 0,
                                                scenario.scenario_id.country_id,
                                                scenario.lanelet_network,
                                                scenario.obstacles,
                                                [])

                cpp_env_model.remove_scenario(i_s)
                # if len(scenario.lanelet_network.intersections) >0:
                #     return
                print("SUCCESS", len(scenario.lanelet_network.intersections))
            except BaseException as e:
                print(e)
                continue




if __name__ == '__main__':
    unittest.main()
