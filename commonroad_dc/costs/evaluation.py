import itertools
from enum import Enum
from typing import Dict, List

from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.costs.route_matcher import LaneletRouteMatcher, SolutionProperties, merge_trajectories

from commonroad.common.solution import Solution, CostFunction, VehicleType

import commonroad_dc.costs.partial_cost_functions as cost_functions


class PartialCostFunction(Enum):
    A = "A"  """Acceleration"""
    J = "J"  """Jerk"""
    Jlat = "Jlat"  """Lateral Jerk"""
    Jlon = "Jlon"  """Longitudinal Jerk"""
    SA = "SA"  """Steering Angle"""
    SR = "SR"  """Steering Rate"""
    Y = "Y"  """Yaw Rate"""
    LC = "LC"  """Lane Center Offset"""
    V = "V"  """Velocity Offset"""
    Vlon = "Vlon"  """Longitudinal Velocity Offset"""
    O = "O"  """Orientation Offset"""
    D = "D"  """Distance to Obstacles"""
    L = "L"  """Path Length"""
    T = "T"  """Time"""
    ID = "ID"  """Inverse Duration"""


class PartialCostFunctionMapping(Enum):
    A = cost_functions.acceleration_cost
    J = cost_functions.jerk_cost
    Jlat = cost_functions.jerk_lat_cost
    Jlon = cost_functions.jerk_lon_cost
    SA = cost_functions.steering_angle_cost
    SR = cost_functions.steering_rate_cost
    Y = cost_functions.yaw_cost
    LC = cost_functions.lane_center_offset_cost
    V = cost_functions.velocity_offset_cost
    Vlon = cost_functions.longitudinal_velocity_offset_cost
    O = cost_functions.orientation_offset_cost
    D = cost_functions.distance_to_obstacle_cost
    L = cost_functions.lane_center_offset_cost
    T = cost_functions.time_cost
    ID = cost_functions.inverse_duration_cost


class CostFunctionMapping(Enum):
    JB1 = [
        (PartialCostFunction.T, PartialCostFunctionMapping.T, 1.0)
    ]
    MW1 = [
        (PartialCostFunction.Jlat, PartialCostFunctionMapping.Jlat, 5.0),
        (PartialCostFunction.Jlon, PartialCostFunctionMapping.Jlon, 0.5),
        (PartialCostFunction.Vlon, PartialCostFunctionMapping.Vlon, 0.2),
        (PartialCostFunction.ID, PartialCostFunctionMapping.ID, 1.0)
    ]
    SA1 = [
        (PartialCostFunction.SA, PartialCostFunctionMapping.SA, 0.1),
        (PartialCostFunction.SR, PartialCostFunctionMapping.SR, 0.1),
        (PartialCostFunction.D, PartialCostFunctionMapping.D, 100000.0),
    ]
    SM1 = [
        (PartialCostFunction.A, PartialCostFunctionMapping.A, 50.0),
        (PartialCostFunction.SA, PartialCostFunctionMapping.SA, 50.0),
        (PartialCostFunction.SR, PartialCostFunctionMapping.SR, 50.0),
        (PartialCostFunction.L, PartialCostFunctionMapping.L, 1.0),
        (PartialCostFunction.V, PartialCostFunctionMapping.V, 20.0),
        (PartialCostFunction.O, PartialCostFunctionMapping.O, 50.0),
    ]
    SM2 = [
        (PartialCostFunction.A, PartialCostFunctionMapping.A, 50.0),
        (PartialCostFunction.SA, PartialCostFunctionMapping.SA, 50.0),
        (PartialCostFunction.SR, PartialCostFunctionMapping.SR, 50.0),
        (PartialCostFunction.L, PartialCostFunctionMapping.L, 1.0),
        (PartialCostFunction.O, PartialCostFunctionMapping.O, 50.0),
    ]
    SM3 = [
        (PartialCostFunction.A, PartialCostFunctionMapping.A, 50.0),
        (PartialCostFunction.SA, PartialCostFunctionMapping.SA, 50.0),
        (PartialCostFunction.SR, PartialCostFunctionMapping.SR, 50.0),
        (PartialCostFunction.V, PartialCostFunctionMapping.V, 20.0),
        (PartialCostFunction.O, PartialCostFunctionMapping.O, 50.0),
    ]
    WX1 = [
        (PartialCostFunction.T, PartialCostFunctionMapping.T, 10.0),
        (PartialCostFunction.V, PartialCostFunctionMapping.V, 1.0),
        (PartialCostFunction.A, PartialCostFunctionMapping.A, 0.1),
        (PartialCostFunction.J, PartialCostFunctionMapping.J, 0.1),
        (PartialCostFunction.D, PartialCostFunctionMapping.D, 0.1),
        (PartialCostFunction.L, PartialCostFunctionMapping.L, 10.0),
    ]


# additional attributes that need to be computed before evaluation
required_properties = {
    PartialCostFunction.A: [],
    PartialCostFunction.J: [],
    PartialCostFunction.Jlat: [SolutionProperties.LatJerk],
    PartialCostFunction.Jlon: [SolutionProperties.LonJerk],
    PartialCostFunction.SA: [],
    PartialCostFunction.SR: [],
    PartialCostFunction.Y: [],
    PartialCostFunction.LC: [],
    PartialCostFunction.V: [],
    PartialCostFunction.Vlon: [SolutionProperties.LonVelocity, SolutionProperties.AllowedVelocityInterval],
    PartialCostFunction.O: [],
    PartialCostFunction.D: [],
    PartialCostFunction.L: [],
    PartialCostFunction.T: [],
    PartialCostFunction.ID: [],
}


class CostFunctionEvaluator:
    def __init__(self, cost_function_id: CostFunction, vehicle_type: VehicleType):
        self.cost_function_id: CostFunction = cost_function_id
        self.vehicle_type = vehicle_type
        self.partial_cost_funcs: CostFunctionMapping = CostFunctionMapping[self.cost_function_id.name]

    @classmethod
    def init_from_string_id(cls, cost_function_id: CostFunction, vehicle_type: VehicleType):
        return cls(cost_function_id=cost_function_id, vehicle_type=vehicle_type)

    @property
    def required_properties(self):
        return list(itertools.chain.from_iterable(required_properties[p[0]] for p in self.partial_cost_funcs.value))

    def evaluate_pp_solution(
            self,
            cr_scenario: Scenario,
            cr_pproblem: PlanningProblem,
            trajectory: Trajectory,
            draw_lanelet_path=False
    ):
        # print([str(s) for s in cr_trajectory.state_list])
        evaluation_result = PlanningProblemCostResult(cost_function_id=self.cost_function_id,
                                                      solution_id=cr_pproblem.planning_problem_id)
        lm = LaneletRouteMatcher(cr_scenario, self.vehicle_type)
        trajectory, _, properties = lm.compute_curvilinear_coordinates(trajectory,
                                                                       required_properties=self.required_properties,
                                                                       draw_lanelet_path=draw_lanelet_path)
        for pcf, pcf_func, weight in self.partial_cost_funcs.value:
            evaluation_result.add_partial_costs(pcf, pcf_func(cr_scenario, cr_pproblem, trajectory, properties),
                                                weight)

        return evaluation_result

    def evaluate_solution(
            self,
            scenario: Scenario,
            cr_pproblems: PlanningProblemSet,
            solution: Solution
    ):
        results = SolutionResult(benchmark_id=solution.benchmark_id)
        for pps in solution.planning_problem_solutions:
            results.add_results(
                self.evaluate_pp_solution(scenario, cr_pproblems.planning_problem_dict[pps.planning_problem_id],
                                          pps.trajectory, False))

        return results


class PlanningProblemCostResult:
    def __init__(self, cost_function_id: CostFunction, solution_id: int):
        self.cost_function_id = cost_function_id
        self.partial_costs: Dict[PartialCostFunction, float] = {}
        self.weights: Dict[PartialCostFunction, float] = {}
        self.solution_id: int = solution_id

    @property
    def total_costs(self) -> float:
        c = 0.0
        for pcf, cost in self.partial_costs.items():
            c += cost * self.weights[pcf]

        return c

    def add_partial_costs(self, pcf: PartialCostFunction, cost: float, weight):
        self.partial_costs[pcf] = cost
        self.weights[pcf] = weight


class SolutionResult:
    def __init__(self, benchmark_id: str, pp_results: List[PlanningProblemCostResult] = ()):
        self.benchmark_id: str = benchmark_id
        self.total_costs: float = 0.0
        self.pp_results: Dict[int, PlanningProblemCostResult] = {}
        for r in pp_results:
            self.add_results(r)

    def add_results(self, pp_result: PlanningProblemCostResult):
        self.pp_results[pp_result.solution_id] = pp_result
        self.total_costs += pp_result.total_costs
