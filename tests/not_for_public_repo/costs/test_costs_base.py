import glob
import os
import unittest
import warnings

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CostFunction, CommonRoadSolutionReader
from commonroad.scenario.scenario import ScenarioID


class TestSolutionEvaluationBase(unittest.TestCase):
    def setUp(self) -> None:
        self.test_ressources_dir = os.path.join(os.path.dirname(__file__), "ressources")
        self.test_scenario_dir = os.path.join(self.test_ressources_dir, "scenarios")
        self.test_solutions_dir = os.path.join(self.test_ressources_dir, "example_solutions")

        self.all_scenario_dir = os.path.normpath(os.path.join(os.path.dirname(__file__),
                                                              "../../../../commonroad-scenarios/scenarios"))
        assert os.path.isdir(self.all_scenario_dir), f"{self.all_scenario_dir} is not a directory. You need to clone" \
                                                     f"https://gitlab.lrz.de/tum-cps/commonroad-scenarios!"
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