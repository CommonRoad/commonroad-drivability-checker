import copy
import math
import timeit
import random

import numpy as np
import unittest
from commonroad.common.solution import VehicleType, Solution, PlanningProblemSolution, VehicleModel, CostFunction
from commonroad.common.util import Interval
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.scenario import ScenarioID
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InputState, PMInputState, PMState, KSState, STState, MBState, \
    CustomState

import commonroad_dc.feasibility.feasibility_checker as feasibility_checker
from commonroad_dc.feasibility.solution_checker import starts_at_correct_state, SolutionCheckerException
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics
from dummy_data_generator import DummyDataGenerator


class TestFeasibilityChecker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.disable_pm_tests = False
        cls.disable_ks_tests = False
        cls.disable_st_tests = True
        cls.disable_mb_tests = True  # ATTENTION: TAKES TOO LONG (MB)
        cls.disable_kst_tests = False

        cls.dt = 0.1
        cls.input_range_sample = 5  # Lower if you want to test less samples (less time)
        cls.velocity_range_sample = 5  # Lower if you want to test less samples (less time)

        cls.pm_dynamics = VehicleDynamics.PM(VehicleType.FORD_ESCORT)
        cls.ks_dynamics = VehicleDynamics.KS(VehicleType.FORD_ESCORT)
        cls.st_dynamics = VehicleDynamics.ST(VehicleType.FORD_ESCORT)
        cls.mb_dynamics = VehicleDynamics.MB(VehicleType.FORD_ESCORT)
        cls.kst_dynamics = VehicleDynamics.KST(VehicleType.TRUCK)

        cls.acceleration_max = cls.ks_dynamics.parameters.longitudinal.a_max
        cls.input_accelerations = np.linspace(-cls.acceleration_max, cls.acceleration_max,
                                              cls.input_range_sample)

        cls.steering_angle_rate_max = cls.ks_dynamics.parameters.steering.v_max
        cls.input_steerings = np.linspace(-cls.steering_angle_rate_max, cls.steering_angle_rate_max,
                                          cls.input_range_sample)

        cls.max_velocity = cls.ks_dynamics.parameters.longitudinal.v_max
        cls.min_velocity = cls.ks_dynamics.parameters.longitudinal.v_min
        cls.velocities = np.linspace(cls.min_velocity, cls.max_velocity, cls.velocity_range_sample)

        cls.zero_init_state = DummyDataGenerator.create_initial_state(velocity=0)
        cls.init_states = [DummyDataGenerator.create_initial_state(velocity) for velocity in cls.velocities]
        cls.pm_init_states = {init_state.velocity: cls.pm_dynamics.convert_initial_state(init_state)
                              for init_state in cls.init_states}
        cls.ks_init_states = {init_state.velocity: cls.ks_dynamics.convert_initial_state(init_state)
                              for init_state in cls.init_states}
        cls.kst_init_states = {init_state.velocity: cls.kst_dynamics.convert_initial_state(init_state)
                              for init_state in cls.init_states}
        cls.st_init_states = {init_state.velocity: cls.st_dynamics.convert_initial_state(init_state)
                              for init_state in cls.init_states}
        cls.mb_init_states = {init_state.velocity: cls.mb_dynamics.convert_initial_state(init_state)
                              for init_state in cls.init_states}

        cls.inputs = [
            InputState(acceleration=acceleration, steering_angle_speed=steering, time_step=0)
            for acceleration in cls.input_accelerations
            for steering in cls.input_steerings
        ]
        cls.pm_inputs = [
            PMInputState(acceleration=acceleration_x, acceleration_y=acceleration_y, time_step=0)
            for acceleration_x in cls.input_accelerations
            for acceleration_y in cls.input_accelerations
        ]

        if not cls.disable_pm_tests:
            cls.pm_next_states = {
                velocity: [cls.pm_dynamics.simulate_next_state(cls.pm_init_states[velocity], inp, cls.dt)
                           for inp in cls.pm_inputs
                           if not cls.pm_dynamics.violates_friction_circle(cls.pm_init_states[velocity], inp)]
                for velocity in cls.velocities
            }

        if not cls.disable_ks_tests:
            cls.ks_next_states = {
                velocity: [cls.ks_dynamics.simulate_next_state(cls.ks_init_states[velocity], inp, cls.dt, False)
                           for inp in cls.inputs
                           if not cls.ks_dynamics.violates_friction_circle(cls.ks_init_states[velocity], inp)]
                for velocity in cls.velocities
            }

        if not cls.disable_kst_tests:
            cls.kst_next_states = {
                velocity: [cls.kst_dynamics.simulate_next_state(cls.kst_init_states[velocity], inp, cls.dt, False)
                           for inp in cls.inputs
                           if not cls.kst_dynamics.violates_friction_circle(cls.kst_init_states[velocity], inp)]
                for velocity in cls.velocities
            }

        if not cls.disable_st_tests:
            cls.st_next_states = {
                velocity: [cls.st_dynamics.simulate_next_state(cls.st_init_states[velocity], inp, cls.dt, False)
                           for inp in cls.inputs
                           if not cls.st_dynamics.violates_friction_circle(cls.st_init_states[velocity], inp)]
                for velocity in cls.velocities
            }

        if not cls.disable_mb_tests:
            cls.mb_next_states = {
                velocity: [cls.mb_dynamics.simulate_next_state(cls.mb_init_states[velocity], inp, cls.dt, False)
                           for inp in cls.inputs
                           if not cls.st_dynamics.violates_friction_circle(cls.st_init_states[velocity], inp)]
                for velocity in cls.velocities
            }

    @staticmethod
    def create_random_float(lower, upper):
        return random.uniform(lower, upper)

    def setUp(self):
        self.zero_pm_init_state = self.pm_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_ks_init_state = self.ks_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_kst_init_state = self.kst_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_st_init_state = self.st_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_mb_init_state = self.mb_dynamics.convert_initial_state(self.zero_init_state)
        self.test_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.error_cases = []
        self.total_time = 0

    def tearDown(self):
        if not self.test_count == 0:
            print(f'Total Tests {self.test_count}, Success: {self.success_count}, Failure: {self.fail_count}, '
                  f'Total Elapsed Time: {self.total_time} seconds')
            print(f'Avg. Time/Test: {self.total_time / self.test_count}')

        if not len(self.error_cases) == 0:
            print('Failure cases:\n')
            for vehicle, init_state, inp, reconst_inp in self.error_cases:
                print(f'Model: {type(vehicle)}\n'
                      f'Init State: {init_state}\n'
                      f'Input: {inp}\n'
                      f'Reconstructed Input: {reconst_inp}\n')

    def test_starts_at_correct_state(self):
        # correct state
        init_state = DummyDataGenerator.create_random_initial_state(pos_min=-100.0, pos_max=100.0,
                                                                    v_min=-5, v_max=5,
                                                                    yaw_rate_max=np.math.pi / 10)
        init_state_solution = copy.deepcopy(init_state)
        traj = Trajectory(0, state_list=[init_state_solution])
        pp_sol = PlanningProblemSolution(1, VehicleModel.KS, VehicleType.FORD_ESCORT, cost_function=CostFunction.SM1,
                                         trajectory=traj)

        solution = Solution(ScenarioID(), [pp_sol])
        pp = PlanningProblem(planning_problem_id=1,
                             initial_state=init_state,
                             goal_region=GoalRegion([CustomState(time_step=Interval(0, 10))]))
        pp_set = PlanningProblemSet([pp])
        starts_at_correct_state(solution, pp_set)

        # alter position
        init_state = DummyDataGenerator.create_random_initial_state(pos_min=-100.0, pos_max=100.0,
                                                                    v_min=-5, v_max=5,
                                                                    yaw_rate_max=np.math.pi / 10)
        init_state_solution = copy.deepcopy(init_state)
        init_state_solution.position[0] = 1000
        traj = Trajectory(0, state_list=[init_state_solution])
        pp_sol = PlanningProblemSolution(1, VehicleModel.KS, VehicleType.FORD_ESCORT, cost_function=CostFunction.SM1,
                                      trajectory=traj)

        solution = Solution(ScenarioID(), [pp_sol])
        pp = PlanningProblem(planning_problem_id=1,
                             initial_state=init_state,
                             goal_region=GoalRegion([CustomState(time_step=Interval(0, 10))]))
        pp_set = PlanningProblemSet([pp])
        try:
            starts_at_correct_state(solution, pp_set)
            exc_raised = False
        except SolutionCheckerException:
            exc_raised = True
        self.assertTrue(exc_raised)

        # alter velocity
        init_state = DummyDataGenerator.create_random_initial_state(pos_min=-100.0, pos_max=100.0,
                                                                    v_min=-5, v_max=5,
                                                                    yaw_rate_max=np.math.pi / 10)
        init_state_solution = copy.deepcopy(init_state)
        init_state_solution.velocity = 1000
        traj = Trajectory(0, state_list=[init_state_solution])
        pp_sol = PlanningProblemSolution(1, VehicleModel.KS, VehicleType.FORD_ESCORT, cost_function=CostFunction.SM1,
                                         trajectory=traj)

        solution = Solution(ScenarioID(), [pp_sol])
        pp = PlanningProblem(planning_problem_id=1,
                             initial_state=init_state,
                             goal_region=GoalRegion([CustomState(time_step=Interval(0, 10))]))
        pp_set = PlanningProblemSet([pp])
        try:
            starts_at_correct_state(solution, pp_set)
            exc_raised = False
        except SolutionCheckerException:
            exc_raised = True
        self.assertTrue(exc_raised)

        # PM model no exception
        init_state = DummyDataGenerator.create_random_initial_state(pos_min=-100.0, pos_max=100.0,
                                                                    v_min=-5, v_max=5,
                                                                    yaw_rate_max=np.math.pi / 10)
        init_state.velocity = abs(init_state.velocity)
        init_state_solution = PMState(position=init_state.position,
                                      time_step=init_state.time_step,
                                      velocity=init_state.velocity * math.cos(init_state.orientation),
                                      velocity_y=init_state.velocity * math.sin(init_state.orientation))

        traj = Trajectory(0, state_list=[init_state_solution])
        pp_sol = PlanningProblemSolution(1, VehicleModel.PM, VehicleType.FORD_ESCORT,
                                         cost_function=CostFunction.JB1,
                                         trajectory=traj)

        solution = Solution(ScenarioID(), [pp_sol])
        pp = PlanningProblem(planning_problem_id=1,
                             initial_state=init_state,
                             goal_region=GoalRegion([CustomState(time_step=Interval(0, 10))]))
        pp_set = PlanningProblemSet([pp])
        starts_at_correct_state(solution, pp_set)

        # PM model exception
        init_state = DummyDataGenerator.create_random_initial_state(pos_min=-100.0, pos_max=100.0,
                                                                    v_min=-5, v_max=5,
                                                                    yaw_rate_max=np.math.pi / 10)
        init_state.velocity = abs(init_state.velocity)
        init_state_solution = PMState(position=init_state.position, time_step=init_state.time_step,
                                      velocity=init_state.velocity * math.cos(init_state.orientation),
                                      velocity_y=init_state.velocity * math.sin(init_state.orientation) + 0.2)

        traj = Trajectory(0, state_list=[init_state_solution])
        pp_sol = PlanningProblemSolution(1, VehicleModel.PM, VehicleType.FORD_ESCORT, cost_function=CostFunction.JB1,
                                         trajectory=traj)

        solution = Solution(ScenarioID(), [pp_sol])
        pp = PlanningProblem(planning_problem_id=1, initial_state=init_state,
                             goal_region=GoalRegion([CustomState(time_step=Interval(0, 10))]))
        pp_set = PlanningProblemSet([pp])
        try:
            starts_at_correct_state(solution, pp_set)

        except SolutionCheckerException as e:
            print(e)
            exc_raised = True
        self.assertTrue(exc_raised)

    def _test_next_states(self, vehicle, init_state, next_states, inputs):
        for idx, next_state in enumerate(next_states):
            start = timeit.default_timer()
            feasible, reconstructed_input = feasibility_checker.state_transition_feasibility(init_state, next_state,
                                                                                             vehicle, self.dt)
            stop = timeit.default_timer()
            self.total_time = self.total_time + stop - start

            if not feasible:
                self.fail_count += 1
                self.error_cases.append((vehicle, init_state, inputs[idx], reconstructed_input))
            else:
                self.success_count += 1

            self.test_count += 1
            if self.test_count % 100 == 0:
                print(f'Tested {self.test_count}, Success: {self.success_count}, Failure: {self.fail_count}, '
                      f'Total Elapsed Time: {self.total_time} seconds')

            # assert feasible

    def test_state_transition_feasibility_pm(self):
        if self.disable_pm_tests:
            return
        for velocity, next_states in self.pm_next_states.items():
            self._test_next_states(self.pm_dynamics, self.pm_init_states[velocity], next_states, self.pm_inputs)

    def test_state_transition_feasibility_pm_velocity_bounds(self):
        if self.disable_pm_tests:
            return
        init_state = self.zero_pm_init_state
        init_state.velocity = self.pm_dynamics.parameters.longitudinal.v_max
        init_state.velocity_y = self.pm_dynamics.parameters.longitudinal.v_max
        next_states = list(filter(None, [self.pm_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.pm_inputs]))

        self._test_next_states(self.pm_dynamics, init_state, next_states, self.pm_inputs)

    def test_state_transition_feasibility_ks(self):
        if self.disable_ks_tests:
            return
        for velocity, next_states in self.ks_next_states.items():
            self._test_next_states(self.ks_dynamics, self.ks_init_states[velocity], next_states, self.inputs)

    def test_state_transition_feasibility_ks_velocity_and_steering_bounds(self):
        if self.disable_ks_tests:
            return
        init_state = self.zero_ks_init_state
        init_state.velocity = self.ks_dynamics.parameters.longitudinal.v_max
        init_state.steering_angle = self.ks_dynamics.parameters.steering.max
        next_states = list(filter(None, [self.ks_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.ks_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_ks_velocity_switch(self):
        if self.disable_ks_tests:
            return
        init_state = self.zero_ks_init_state
        init_state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch
        next_states = list(filter(None, [self.ks_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.ks_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_ks_velocity_switch_below(self):
        if self.disable_ks_tests:
            return
        init_state = self.zero_ks_init_state
        init_state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch - 0.001
        next_states = list(filter(None, [self.ks_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.ks_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_ks_velocity_switch_above(self):
        if self.disable_ks_tests:
            return
        init_state = self.zero_ks_init_state
        init_state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch + 0.001
        next_states = list(filter(None, [self.ks_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.ks_dynamics, init_state, next_states, self.inputs)


    def test_state_transition_feasibility_kst_velocity_switch_below(self):
        if self.disable_kst_tests:
            return
        init_state = self.zero_kst_init_state
        init_state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch - 0.001
        next_states = list(filter(None, [self.kst_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.kst_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_kst_velocity_switch_above(self):
        if self.disable_kst_tests:
            return
        init_state = self.zero_kst_init_state
        init_state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch + 0.001
        next_states = list(filter(None, [self.kst_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.kst_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_kst(self):
        if self.disable_kst_tests:
            return
        for velocity, next_states in self.kst_next_states.items():
            self._test_next_states(self.kst_dynamics, self.kst_init_states[velocity], next_states, self.inputs)

    def test_state_transition_feasibility_kst_velocity_and_steering_bounds(self):
        if self.disable_kst_tests:
            return
        init_state = self.zero_kst_init_state
        init_state.velocity = self.kst_dynamics.parameters.longitudinal.v_max
        init_state.steering_angle = self.kst_dynamics.parameters.steering.max
        next_states = list(filter(None, [self.kst_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.kst_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_kst_velocity_switch(self):
        if self.disable_kst_tests:
            return
        init_state = self.zero_kst_init_state
        init_state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch
        next_states = list(filter(None, [self.kst_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.kst_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st(self):
        if self.disable_st_tests:
            return
        for velocity, next_states in self.st_next_states.items():
            self._test_next_states(self.st_dynamics, self.st_init_states[velocity], next_states, self.inputs)

    def test_state_transition_feasibility_st_velocity_and_steering_bounds(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = self.st_dynamics.parameters.longitudinal.v_max
        init_state.steering_angle = self.st_dynamics.parameters.steering.min
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_velocity_switch(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = self.st_dynamics.parameters.longitudinal.v_switch
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_velocity_switch_below(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = self.st_dynamics.parameters.longitudinal.v_switch - 0.001
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_velocity_switch_above(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = self.st_dynamics.parameters.longitudinal.v_switch + 0.001
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_low_velocity(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = 0.1
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_low_velocity_below(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = 0.09
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_st_low_velocity_above(self):
        if self.disable_st_tests:
            return
        init_state = self.zero_st_init_state
        init_state.velocity = 0.11
        next_states = list(filter(None, [self.st_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.st_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb(self):
        if self.disable_mb_tests:
            return
        for velocity, next_states in self.mb_next_states.items():
            self._test_next_states(self.mb_dynamics, self.mb_init_states[velocity], next_states, self.inputs)

    def test_state_transition_feasibility_mb_velocity_and_steering_bounds(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = self.mb_dynamics.parameters.longitudinal.v_max
        init_state.steering_angle = self.mb_dynamics.parameters.steering.min
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_velocity_switch(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_velocity_switch_below(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch - 0.001
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_velocity_switch_above(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch + 0.001
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_low_velocity(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = 0.1
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_low_velocity_below(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = 0.09
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def test_state_transition_feasibility_mb_low_velocity_above(self):
        if self.disable_mb_tests:
            return
        init_state = self.zero_mb_init_state
        init_state.velocity = 0.13
        next_states = list(filter(None, [self.mb_dynamics.simulate_next_state(init_state, inp, self.dt, False)
                                         for inp in self.inputs]))

        self._test_next_states(self.mb_dynamics, init_state, next_states, self.inputs)

    def _test_trajectory_feasibility(self, vehicle, init_state, trajectory_generator):
        trajectory, input_vector = trajectory_generator(init_state, vehicle)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory, vehicle, self.dt)
        assert feasible

    def test_trajectory_feasibility_pm(self):
        if self.disable_pm_tests:
            return
        trajectory, input_vector = DummyDataGenerator.create_random_pm_trajectory(self.zero_pm_init_state,
                                                                                  self.pm_dynamics,
                                                                                  self.dt, 20)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory,
                                                                                    self.pm_dynamics,
                                                                                    self.dt)
        assert feasible

    def test_trajectory_feasibility_ks(self):
        if self.disable_ks_tests:
            return
        trajectory, input_vector = DummyDataGenerator.create_random_trajectory(self.zero_ks_init_state,
                                                                               self.ks_dynamics,
                                                                               self.dt, 20)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory,
                                                                                    self.ks_dynamics,
                                                                                    self.dt)
        assert feasible

    def test_trajectory_feasibility_kst(self):
        if self.disable_kst_tests:
            return
        trajectory, input_vector = DummyDataGenerator.create_random_trajectory(self.zero_kst_init_state,
                                                                               self.kst_dynamics,
                                                                               self.dt, 20)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory,
                                                                                    self.kst_dynamics,
                                                                                    self.dt)
        assert feasible

    def test_trajectory_feasibility_st(self):
        if self.disable_st_tests:
            return
        trajectory, input_vector = DummyDataGenerator.create_random_trajectory(self.zero_st_init_state,
                                                                               self.st_dynamics,
                                                                               self.dt, 20)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory,
                                                                                    self.st_dynamics,
                                                                                    self.dt)
        assert feasible

    def test_trajectory_feasibility_mb(self):
        if self.disable_mb_tests:
            return
        trajectory, input_vector = DummyDataGenerator.create_random_trajectory(self.zero_mb_init_state,
                                                                               self.mb_dynamics,
                                                                               self.dt, 20)
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory,
                                                                                    self.mb_dynamics,
                                                                                    self.dt)
        assert feasible

    def test_pm_input_vector_feasibility(self):
        trajectory, input_vector = DummyDataGenerator.create_random_pm_trajectory(self.zero_pm_init_state,
                                                                                  self.pm_dynamics,
                                                                                  self.dt, 20)

        assert feasibility_checker.input_vector_feasibility(self.zero_pm_init_state, input_vector,
                                                            self.pm_dynamics, self.dt)[0]

    def test_input_vector_feasibility(self):
        if not self.disable_ks_tests:
            _, ks_vector = DummyDataGenerator.create_random_trajectory(self.zero_ks_init_state,
                                                                       self.ks_dynamics,
                                                                       self.dt, 20)
            assert feasibility_checker.input_vector_feasibility(self.zero_ks_init_state, ks_vector,
                                                                self.ks_dynamics, self.dt)[0]
        if not self.disable_kst_tests:
            _, kst_vector = DummyDataGenerator.create_random_trajectory(self.zero_kst_init_state,
                                                                        self.kst_dynamics,
                                                                        self.dt, 20)
            assert feasibility_checker.input_vector_feasibility(self.zero_kst_init_state, kst_vector,
                                                                self.kst_dynamics, self.dt)[0]
        if not self.disable_st_tests:
            _, st_vector = DummyDataGenerator.create_random_trajectory(self.zero_st_init_state,
                                                                       self.st_dynamics,
                                                                       self.dt, 20)
            assert feasibility_checker.input_vector_feasibility(self.zero_st_init_state, st_vector,
                                                                self.st_dynamics, self.dt)[0]
        if not self.disable_mb_tests:
            _, mb_vector = DummyDataGenerator.create_random_trajectory(self.zero_mb_init_state,
                                                                       self.mb_dynamics,
                                                                       self.dt, 20)
            assert feasibility_checker.input_vector_feasibility(self.zero_mb_init_state, mb_vector,
                                                                self.mb_dynamics, self.dt)[0]


if __name__ == '__main__':
    unittest.main()
