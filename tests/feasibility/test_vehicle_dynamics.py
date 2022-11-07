import math
import numpy as np
import unittest
import pickle
from commonroad.common.solution import VehicleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InputState, PMInputState, LKSInputState
from scipy.integrate import odeint

from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics, LinearizedKSDynamics
from dummy_data_generator import DummyDataGenerator


class TestVehicleDynamics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Disable Test flags
        cls.disable_pm_tests = False
        cls.disable_ks_tests = False
        cls.disable_kst_tests = False
        cls.disable_st_tests = False
        cls.disable_mb_tests = True
        cls.disable_lks_tests = False

        # general test parameters
        cls.dt = 0.1
        cls.input_range_sample = 5  # Lower if you want to test less samples (less time)
        cls.velocity_range_sample = 5  # Lower if you want to test less samples (less time)


        # *********************VehicleDynamics**************************
        # instantiate PM, KS, KST, ST, MB VehicleDynamics
        cls.pm_dynamics = VehicleDynamics.PM(VehicleType.FORD_ESCORT)
        cls.ks_dynamics = VehicleDynamics.KS(VehicleType.FORD_ESCORT)
        cls.kst_dynamics = VehicleDynamics.KST(VehicleType.TRUCK)
        cls.st_dynamics = VehicleDynamics.ST(VehicleType.FORD_ESCORT)
        cls.mb_dynamics = VehicleDynamics.MB(VehicleType.FORD_ESCORT)

        # instantiate LKS VehicleDynamics
        # load precomputed positions and orientations of reference path (here the reference path is simply a straight line)
        with open('ref_pos_ref_theta.pkl', "rb") as f:
            ref_pos = pickle.load(f)
            ref_theta = pickle.load(f)
        cls.lks_dynamics = VehicleDynamics.LKS(VehicleType.BMW_320i, ref_pos=ref_pos, ref_theta=ref_theta)


        # *********************Initial States**************************
        # create zero and random init state for PM, KS, KS, KST, ST, MB models
        cls.zero_init_state = DummyDataGenerator.create_zero_initial_state()
        cls.random_init_state = DummyDataGenerator.create_random_initial_state()

        # create zero and random init state for LKS model
        cls.zero_init_state_lks = DummyDataGenerator.create_zero_initial_state_lks()
        cls.random_init_state_lks = DummyDataGenerator.create_random_initial_state_lks()


        # *********************Input States**************************
        # inputs for PM, KS, KST, ST, MB
        cls.acceleration_max = cls.ks_dynamics.parameters.longitudinal.a_max
        cls.input_accelerations = np.linspace(-cls.acceleration_max, cls.acceleration_max,
                                              cls.input_range_sample)

        cls.steering_angle_rate_max = cls.ks_dynamics.parameters.steering.v_max
        cls.input_steerings = np.linspace(-cls.steering_angle_rate_max, cls.steering_angle_rate_max,
                                          cls.input_range_sample)

        # inputs for LKS
        cls.jerk_dot_max = cls.lks_dynamics.parameters.longitudinal.j_dot_max
        cls.input_jerk_dots = np.linspace(-cls.jerk_dot_max, cls.jerk_dot_max,
                                          cls.input_range_sample)

        cls.kappa_dot_dot_max = cls.lks_dynamics.parameters.steering.kappa_dot_dot_max
        cls.input_kappa_dot_dots = np.linspace(-cls.kappa_dot_dot_max, cls.kappa_dot_dot_max,
                                               cls.input_range_sample)

        cls.max_velocity = cls.ks_dynamics.parameters.longitudinal.v_max
        cls.min_velocity = cls.ks_dynamics.parameters.longitudinal.v_min
        cls.velocities = np.linspace(cls.min_velocity, cls.max_velocity, cls.velocity_range_sample)

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
        cls.lks_inputs = [
            LKSInputState(jerk_dot=jerk_dot, kappa_dot_dot=kappa_dot_dot, time_step=0)
            for jerk_dot in cls.input_jerk_dots
            for kappa_dot_dot in cls.input_kappa_dot_dots
        ]


        # *********************Random States**************************
        # generate random states
        cls.random_pm_state = DummyDataGenerator.create_random_pm_state()
        cls.random_ks_state = DummyDataGenerator.create_random_ks_state()
        cls.random_kst_state = DummyDataGenerator.create_random_kst_state()
        cls.random_st_state = DummyDataGenerator.create_random_st_state()
        cls.random_mb_state = DummyDataGenerator.create_random_mb_state(cls.mb_dynamics.parameters)
        cls.random_lks_state = DummyDataGenerator.create_random_lks_state()


        # *********************Random Inputs**************************
        # generate random inputs
        cls.random_pm_input = DummyDataGenerator.create_random_pm_input(cls.pm_dynamics.parameters.longitudinal.a_max)
        cls.random_ks_input = DummyDataGenerator.create_random_input(cls.ks_dynamics.parameters.longitudinal.a_max,
                                                                     cls.ks_dynamics.parameters.steering.v_max
                                                                     )
        cls.random_kst_input = DummyDataGenerator.create_random_input(cls.kst_dynamics.parameters.longitudinal.a_max,
                                                                      cls.kst_dynamics.parameters.steering.v_max
                                                                      )
        cls.random_st_input = DummyDataGenerator.create_random_input(cls.st_dynamics.parameters.longitudinal.a_max,
                                                                     cls.st_dynamics.parameters.steering.v_max
                                                                     )
        cls.random_lks_input = DummyDataGenerator.create_random_lks_input(
            cls.lks_dynamics.parameters.longitudinal.j_dot_max,
            cls.lks_dynamics.parameters.steering.kappa_dot_dot_max)

    def setUp(self):
        self.zero_pm_init_state = self.pm_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_ks_init_state = self.ks_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_kst_init_state = self.kst_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_st_init_state = self.st_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_mb_init_state = self.mb_dynamics.convert_initial_state(self.zero_init_state)
        self.zero_lks_init_state = self.zero_init_state_lks
        self.random_pm_init_state = self.pm_dynamics.convert_initial_state(self.random_init_state)
        self.random_ks_init_state = self.ks_dynamics.convert_initial_state(self.random_init_state)
        self.random_kst_init_state = self.kst_dynamics.convert_initial_state(self.random_init_state)
        self.random_st_init_state = self.st_dynamics.convert_initial_state(self.random_init_state)
        self.random_mb_init_state = self.mb_dynamics.convert_initial_state(self.random_init_state)
        self.random_lks_init_state = self.random_init_state_lks
        self.test_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.error_cases = []
        self.total_time = 0

    def test_state_to_array_pm(self):
        state_values, ts = self.pm_dynamics.state_to_array(self.random_pm_state)

        assert state_values[0] == self.random_pm_state.position[0]
        assert state_values[1] == self.random_pm_state.position[1]
        assert state_values[2] == self.random_pm_state.velocity
        assert state_values[3] == self.random_pm_state.velocity_y
        assert ts == self.random_pm_state.time_step

    def test_state_to_array_ks(self):
        state_values, ts = self.ks_dynamics.state_to_array(self.random_ks_state)

        assert state_values[0] == self.random_ks_state.position[0] - self.ks_dynamics.parameters.b * \
               math.cos(self.random_ks_state.orientation)
        assert state_values[1] == self.random_ks_state.position[1] - self.ks_dynamics.parameters.b * \
               math.sin(self.random_ks_state.orientation)
        assert state_values[2] == self.random_ks_state.steering_angle
        assert state_values[3] == self.random_ks_state.velocity
        assert state_values[4] == self.random_ks_state.orientation
        assert ts == self.random_ks_state.time_step

    def test_state_to_array_kst(self):
        state_values, ts = self.kst_dynamics.state_to_array(self.random_kst_state)

        assert state_values[0] == self.random_kst_state.position[0]
        assert state_values[1] == self.random_kst_state.position[1]
        assert state_values[2] == self.random_kst_state.steering_angle
        assert state_values[3] == self.random_kst_state.velocity
        assert state_values[4] == self.random_kst_state.orientation
        assert state_values[5] == self.random_kst_state.hitch_angle
        assert ts == self.random_kst_state.time_step

    def test_state_to_array_st(self):
        state_values, ts = self.st_dynamics.state_to_array(self.random_st_state)

        assert state_values[0] == self.random_st_state.position[0]
        assert state_values[1] == self.random_st_state.position[1]
        assert state_values[2] == self.random_st_state.steering_angle
        assert state_values[3] == self.random_st_state.velocity
        assert state_values[4] == self.random_st_state.orientation
        assert state_values[5] == self.random_st_state.yaw_rate
        assert state_values[6] == self.random_st_state.slip_angle
        assert ts == self.random_st_state.time_step

    def test_state_to_array_mb(self):
        state_values, ts = self.mb_dynamics.state_to_array(self.random_mb_state)

        assert state_values[0] == self.random_mb_state.position[0]
        assert state_values[1] == self.random_mb_state.position[1]
        assert state_values[2] == self.random_mb_state.steering_angle
        assert state_values[3] == self.random_mb_state.velocity
        assert state_values[4] == self.random_mb_state.orientation
        assert state_values[5] == self.random_mb_state.yaw_rate
        assert state_values[6] == self.random_mb_state.roll_angle
        assert state_values[7] == self.random_mb_state.roll_rate
        assert state_values[8] == self.random_mb_state.pitch_angle
        assert state_values[9] == self.random_mb_state.pitch_rate
        assert state_values[10] == self.random_mb_state.velocity_y
        assert state_values[11] == self.random_mb_state.position_z
        assert state_values[12] == self.random_mb_state.velocity_z
        assert state_values[13] == self.random_mb_state.roll_angle_front
        assert state_values[14] == self.random_mb_state.roll_rate_front
        assert state_values[15] == self.random_mb_state.velocity_y_front
        assert state_values[16] == self.random_mb_state.position_z_front
        assert state_values[17] == self.random_mb_state.velocity_z_front
        assert state_values[18] == self.random_mb_state.roll_angle_rear
        assert state_values[19] == self.random_mb_state.roll_rate_rear
        assert state_values[20] == self.random_mb_state.velocity_y_rear
        assert state_values[21] == self.random_mb_state.position_z_rear
        assert state_values[22] == self.random_mb_state.velocity_z_rear
        assert state_values[23] == self.random_mb_state.left_front_wheel_angular_speed
        assert state_values[24] == self.random_mb_state.right_front_wheel_angular_speed
        assert state_values[25] == self.random_mb_state.left_rear_wheel_angular_speed
        assert state_values[26] == self.random_mb_state.right_rear_wheel_angular_speed
        assert state_values[27] == self.random_mb_state.delta_y_f
        assert state_values[28] == self.random_mb_state.delta_y_r
        assert ts == self.random_mb_state.time_step

    def test_state_to_array_lks(self):
        state_values, ts = self.lks_dynamics.state_to_array(self.random_lks_state)
        lon_state = self.random_lks_state[0]
        lat_state = self.random_lks_state[1]

        assert state_values[0] == lon_state.longitudinal_position
        assert state_values[1] == lon_state.velocity
        assert state_values[2] == lon_state.acceleration
        assert state_values[3] == lon_state.jerk
        assert state_values[4] == lat_state.lateral_position
        assert state_values[5] == lat_state.orientation
        assert state_values[6] == lat_state.curvature
        assert state_values[7] == lat_state.curvature_rate
        assert ts == lon_state.time_step

    def test_initial_state_to_array_pm(self):
        velocity_x = math.cos(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y = math.sin(self.random_init_state.orientation) * self.random_init_state.velocity

        state_values, ts = self.pm_dynamics.state_to_array(self.random_init_state)

        assert state_values[0] == self.random_init_state.position[0]
        assert state_values[1] == self.random_init_state.position[1]
        assert state_values[2] == velocity_x
        assert state_values[3] == velocity_y
        assert ts == self.random_init_state.time_step

    def test_initial_state_to_array_ks(self):
        state_values, ts = self.ks_dynamics.state_to_array(self.random_init_state)

        assert state_values[0] == self.random_init_state.position[0] - self.ks_dynamics.parameters.b * \
               math.cos(self.random_init_state.orientation)
        assert state_values[1] == self.random_init_state.position[1] - self.ks_dynamics.parameters.b * \
               math.sin(self.random_init_state.orientation)
        assert state_values[2] == 0.0  # state.steering_angle
        assert state_values[3] == self.random_init_state.velocity
        assert state_values[4] == self.random_init_state.orientation
        assert ts == self.random_init_state.time_step

    def test_initial_state_to_array_kst(self):
        state_values, ts = self.kst_dynamics.state_to_array(self.random_init_state)

        assert state_values[0] == self.random_init_state.position[0]
        assert state_values[1] == self.random_init_state.position[1]
        assert state_values[2] == 0.0  # state.steering_angle
        assert state_values[3] == self.random_init_state.velocity
        assert state_values[4] == self.random_init_state.orientation
        assert state_values[5] == 0.0  # state.hitch_angle
        assert ts == self.random_init_state.time_step

    def test_initial_state_to_array_st(self):
        state_values, ts = self.st_dynamics.state_to_array(self.random_init_state)

        assert state_values[0] == self.random_init_state.position[0]
        assert state_values[1] == self.random_init_state.position[1]
        assert state_values[2] == 0.0  # state.steering_angle
        assert state_values[3] == self.random_init_state.velocity
        assert state_values[4] == self.random_init_state.orientation
        assert state_values[5] == self.random_init_state.yaw_rate
        assert state_values[6] == self.random_init_state.slip_angle
        assert ts == self.random_init_state.time_step

    def test_initial_state_to_array_mb(self):
        p = self.mb_dynamics.parameters
        g = 9.81  # [m/s^2]
        F0_z_f = p.m_s * g * p.b / (p.a + p.b) + p.m_uf * g
        F0_z_r = p.m_s * g * p.a / (p.a + p.b) + p.m_ur * g
        position_z_front = F0_z_f / 2 * p.K_zt
        position_z_rear = F0_z_r / 2 * p.K_zt
        velocity_x = math.cos(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y = math.sin(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y_front = velocity_y + p.a * self.random_init_state.yaw_rate
        velocity_y_rear = velocity_y - p.b * self.random_init_state.yaw_rate
        wheel_speed = velocity_x / p.R_w

        state_values, ts = self.mb_dynamics.state_to_array(self.random_init_state)

        assert state_values[0] == self.random_init_state.position[0]
        assert state_values[1] == self.random_init_state.position[1]
        assert state_values[2] == 0.0  # state.steering_angle
        assert state_values[3] == velocity_x
        assert state_values[4] == self.random_init_state.orientation
        assert state_values[5] == self.random_init_state.yaw_rate
        assert state_values[6] == 0.0  # state.roll_angle
        assert state_values[7] == 0.0  # state.roll_rate
        assert state_values[8] == 0.0  # state.pitch_angle
        assert state_values[9] == 0.0  # state.pitch_rate
        assert state_values[10] == velocity_y
        assert state_values[11] == 0.0  # state.position_z
        assert state_values[12] == 0.0  # state.velocity_z
        assert state_values[13] == 0.0  # state.roll_angle_front
        assert state_values[14] == 0.0  # state.roll_rate_front
        assert state_values[15] == velocity_y_front
        assert state_values[16] == position_z_front
        assert state_values[17] == 0.0  # state.velocity_z_front
        assert state_values[18] == 0.0  # state.roll_angle_rear
        assert state_values[19] == 0.0  # state.roll_rate_rear
        assert state_values[20] == velocity_y_rear
        assert state_values[21] == position_z_rear
        assert state_values[22] == 0.0  # state.velocity_z_rear
        assert state_values[23] == wheel_speed  # state.left_front_wheel_angular_speed
        assert state_values[24] == wheel_speed  # state.right_front_wheel_angular_speed
        assert state_values[25] == wheel_speed  # state.left_rear_wheel_angular_speed
        assert state_values[26] == wheel_speed  # state.right_rear_wheel_angular_speed
        assert state_values[27] == 0.0  # state.delta_y_f
        assert state_values[28] == 0.0  # state.delta_y_r
        assert ts == self.random_init_state.time_step

    def test_array_to_state_pm(self):
        state_values, ts = self.pm_dynamics.state_to_array(self.random_pm_state)

        converted_state = self.pm_dynamics.array_to_state(state_values, ts)

        assert np.all(converted_state.position == self.random_pm_state.position)
        assert converted_state.velocity == self.random_pm_state.velocity
        assert converted_state.velocity_y == self.random_pm_state.velocity_y
        assert ts == self.random_pm_state.time_step

    def test_array_to_state_ks(self):
        state_values, ts = self.ks_dynamics.state_to_array(self.random_ks_state)

        converted_state = self.ks_dynamics.array_to_state(state_values, ts)

        assert np.all(converted_state.position == self.random_ks_state.position)
        assert converted_state.steering_angle == self.random_ks_state.steering_angle
        assert converted_state.velocity == self.random_ks_state.velocity
        assert converted_state.orientation == self.random_ks_state.orientation
        assert ts == self.random_ks_state.time_step

    def test_array_to_state_kst(self):
        state_values, ts = self.kst_dynamics.state_to_array(self.random_kst_state)

        converted_state = self.kst_dynamics.array_to_state(state_values, ts)

        assert np.all(converted_state.position == self.random_kst_state.position)
        assert converted_state.steering_angle == self.random_kst_state.steering_angle
        assert converted_state.velocity == self.random_kst_state.velocity
        assert converted_state.orientation == self.random_kst_state.orientation
        assert ts == self.random_kst_state.time_step

    def test_array_to_state_st(self):
        state_values, ts = self.st_dynamics.state_to_array(self.random_st_state)

        converted_state = self.st_dynamics.array_to_state(state_values, ts)

        assert np.all(converted_state.position == self.random_st_state.position)
        assert converted_state.steering_angle == self.random_st_state.steering_angle
        assert converted_state.velocity == self.random_st_state.velocity
        assert converted_state.orientation == self.random_st_state.orientation
        assert converted_state.yaw_rate == self.random_st_state.yaw_rate
        assert converted_state.slip_angle == self.random_st_state.slip_angle
        assert ts == self.random_st_state.time_step

    def test_array_to_state_mb(self):
        state_values, ts = self.mb_dynamics.state_to_array(self.random_mb_state)

        converted_state = self.mb_dynamics.array_to_state(state_values, ts)

        assert np.all(converted_state.position == self.random_mb_state.position)
        assert converted_state.steering_angle == self.random_mb_state.steering_angle
        assert converted_state.velocity == self.random_mb_state.velocity
        assert converted_state.orientation == self.random_mb_state.orientation
        assert converted_state.yaw_rate == self.random_mb_state.yaw_rate
        assert converted_state.roll_angle == self.random_mb_state.roll_angle
        assert converted_state.roll_rate == self.random_mb_state.roll_rate
        assert converted_state.pitch_angle == self.random_mb_state.pitch_angle
        assert converted_state.pitch_rate == self.random_mb_state.pitch_rate
        assert converted_state.velocity_y == self.random_mb_state.velocity_y
        assert converted_state.position_z == self.random_mb_state.position_z
        assert converted_state.velocity_z == self.random_mb_state.velocity_z
        assert converted_state.roll_angle_front == self.random_mb_state.roll_angle_front
        assert converted_state.roll_rate_front == self.random_mb_state.roll_rate_front
        assert converted_state.velocity_y_front == self.random_mb_state.velocity_y_front
        assert converted_state.position_z_front == self.random_mb_state.position_z_front
        assert converted_state.velocity_z_front == self.random_mb_state.velocity_z_front
        assert converted_state.roll_angle_rear == self.random_mb_state.roll_angle_rear
        assert converted_state.roll_rate_rear == self.random_mb_state.roll_rate_rear
        assert converted_state.velocity_y_rear == self.random_mb_state.velocity_y_rear
        assert converted_state.position_z_rear == self.random_mb_state.position_z_rear
        assert converted_state.velocity_z_rear == self.random_mb_state.velocity_z_rear
        assert converted_state.left_front_wheel_angular_speed == self.random_mb_state.left_front_wheel_angular_speed
        assert converted_state.right_front_wheel_angular_speed == self.random_mb_state.right_front_wheel_angular_speed
        assert converted_state.left_rear_wheel_angular_speed == self.random_mb_state.left_rear_wheel_angular_speed
        assert converted_state.right_rear_wheel_angular_speed == self.random_mb_state.right_rear_wheel_angular_speed
        assert converted_state.delta_y_f == self.random_mb_state.delta_y_f
        assert converted_state.delta_y_r == self.random_mb_state.delta_y_r
        assert ts == self.random_mb_state.time_step

    def test_array_to_state_lks(self):
        state_values, ts = self.lks_dynamics.state_to_array(self.random_lks_state)
        lon_state = self.random_lks_state[0]
        lat_state = self.random_lks_state[1]

        converted_state = self.lks_dynamics.array_to_state(state_values, ts)
        converted_lon_state = converted_state[0]
        converted_lat_state = converted_state[1]

        assert converted_lon_state.longitudinal_position == lon_state.longitudinal_position
        assert converted_lon_state.velocity == lon_state.velocity
        assert converted_lon_state.acceleration == lon_state.acceleration
        assert converted_lon_state.jerk == lon_state.jerk
        assert converted_lat_state.lateral_position == lat_state.lateral_position
        assert converted_lat_state.orientation == lat_state.orientation
        assert converted_lat_state.curvature == lat_state.curvature
        assert converted_lat_state.curvature_rate == lat_state.curvature_rate
        assert ts == lon_state.time_step

    def test_input_to_array(self):
        input_values, ts = self.ks_dynamics.input_to_array(self.random_ks_input)

        assert input_values[0] == self.random_ks_input.steering_angle_speed
        assert input_values[1] == self.random_ks_input.acceleration
        assert ts == self.random_ks_input.time_step

    def test_pm_input_to_array(self):
        input_values, ts = self.pm_dynamics.input_to_array(self.random_pm_input)

        assert input_values[0] == self.random_pm_input.acceleration
        assert input_values[1] == self.random_pm_input.acceleration_y
        assert ts == self.random_pm_input.time_step

    def test_lks_input_to_array(self):
        input_values, ts = self.lks_dynamics.input_to_array(self.random_lks_input)

        assert input_values[0] == self.random_lks_input.jerk_dot
        assert input_values[1] == self.random_lks_input.kappa_dot_dot
        assert ts == self.random_lks_input.time_step

    def test_array_to_input(self):
        input_values, ts = self.ks_dynamics.input_to_array(self.random_ks_input)

        converted_input = self.ks_dynamics.array_to_input(input_values, ts)

        assert converted_input.steering_angle_speed == self.random_ks_input.steering_angle_speed
        assert converted_input.acceleration == self.random_ks_input.acceleration
        assert ts == self.random_ks_input.time_step

    def test_array_to_pm_input(self):
        input_values, ts = self.pm_dynamics.input_to_array(self.random_pm_input)

        converted_input = self.pm_dynamics.array_to_input(input_values, ts)

        assert converted_input.acceleration == self.random_pm_input.acceleration
        assert converted_input.acceleration_y == self.random_pm_input.acceleration_y
        assert ts == self.random_pm_input.time_step

    def test_array_to_lks_input(self):
        input_values, ts = self.lks_dynamics.input_to_array(self.random_lks_input)

        converted_input = self.lks_dynamics.array_to_input(input_values, ts)

        assert converted_input.jerk_dot == self.random_lks_input.jerk_dot
        assert converted_input.kappa_dot_dot == self.random_lks_input.kappa_dot_dot
        assert ts == self.random_lks_input.time_step

    def test_convert_initial_state_pm(self):
        velocity_x = math.cos(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y = math.sin(self.random_init_state.orientation) * self.random_init_state.velocity

        state = self.pm_dynamics.convert_initial_state(self.random_init_state)

        assert self.random_init_state.position[0] == state.position[0]
        assert self.random_init_state.position[1] == state.position[1]
        assert velocity_x == state.velocity
        assert velocity_y == state.velocity_y
        assert self.random_init_state.time_step == state.time_step

    def test_convert_initial_state_ks(self):
        state = self.ks_dynamics.convert_initial_state(self.random_init_state)

        assert self.random_init_state.position[0] == state.position[0]
        assert self.random_init_state.position[1] == state.position[1]
        assert 0.0 == state.steering_angle
        assert self.random_init_state.velocity == state.velocity
        assert self.random_init_state.orientation == state.orientation
        assert self.random_init_state.time_step == state.time_step

    def test_convert_initial_state_kst(self):
        state = self.kst_dynamics.convert_initial_state(self.random_init_state)

        assert self.random_init_state.position[0] == state.position[0]
        assert self.random_init_state.position[1] == state.position[1]
        assert 0.0 == state.steering_angle
        assert self.random_init_state.velocity == state.velocity
        assert self.random_init_state.orientation == state.orientation
        assert 0.0 == state.hitch_angle
        assert self.random_init_state.time_step == state.time_step

    def test_convert_initial_state_st(self):
        state = self.st_dynamics.convert_initial_state(self.random_init_state)

        assert self.random_init_state.position[0] == state.position[0]
        assert self.random_init_state.position[1] == state.position[1]
        assert 0.0 == state.steering_angle  # state.steering_angle
        assert self.random_init_state.velocity == state.velocity
        assert self.random_init_state.orientation == state.orientation
        assert self.random_init_state.yaw_rate == state.yaw_rate
        assert self.random_init_state.slip_angle == state.slip_angle
        assert self.random_init_state.time_step == state.time_step

    def test_convert_initial_state_mb(self):
        p = self.mb_dynamics.parameters
        g = 9.81  # [m/s^2]
        F0_z_f = p.m_s * g * p.b / (p.a + p.b) + p.m_uf * g
        F0_z_r = p.m_s * g * p.a / (p.a + p.b) + p.m_ur * g
        position_z_front = F0_z_f / 2 * p.K_zt
        position_z_rear = F0_z_r / 2 * p.K_zt
        velocity_x = math.cos(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y = math.sin(self.random_init_state.orientation) * self.random_init_state.velocity
        velocity_y_front = velocity_y + p.a * self.random_init_state.yaw_rate
        velocity_y_rear = velocity_y - p.b * self.random_init_state.yaw_rate
        wheel_speed = velocity_x / p.R_w

        state = self.mb_dynamics.convert_initial_state(self.random_init_state)

        assert self.random_init_state.position[0] == state.position[0]
        assert self.random_init_state.position[1] == state.position[1]
        assert 0.0 == state.steering_angle
        assert velocity_x == state.velocity
        assert self.random_init_state.orientation == state.orientation
        assert self.random_init_state.yaw_rate == state.yaw_rate
        assert 0.0 == state.roll_angle
        assert 0.0 == state.roll_rate
        assert 0.0 == state.pitch_angle
        assert 0.0 == state.pitch_rate
        assert velocity_y == state.velocity_y
        assert 0.0 == state.position_z
        assert 0.0 == state.velocity_z
        assert 0.0 == state.roll_angle_front
        assert 0.0 == state.roll_rate_front
        assert velocity_y_front == state.velocity_y_front
        assert position_z_front == state.position_z_front
        assert 0.0 == state.velocity_z_front
        assert 0.0 == state.roll_angle_rear
        assert 0.0 == state.roll_rate_rear
        assert velocity_y_rear == state.velocity_y_rear
        assert position_z_rear == state.position_z_rear
        assert 0.0 == state.velocity_z_front
        assert wheel_speed == state.left_front_wheel_angular_speed
        assert wheel_speed == state.right_front_wheel_angular_speed
        assert wheel_speed == state.left_rear_wheel_angular_speed
        assert wheel_speed == state.right_rear_wheel_angular_speed
        assert 0.0 == state.delta_y_f
        assert 0.0 == state.delta_y_r
        assert self.random_init_state.time_step == state.time_step

    def test_input_bounds_pm(self):
        assert self.pm_dynamics.input_bounds.lb[0] == -self.pm_dynamics.parameters.longitudinal.a_max
        assert self.pm_dynamics.input_bounds.ub[0] == self.pm_dynamics.parameters.longitudinal.a_max
        assert self.pm_dynamics.input_bounds.lb[1] == -self.pm_dynamics.parameters.longitudinal.a_max
        assert self.pm_dynamics.input_bounds.ub[1] == self.pm_dynamics.parameters.longitudinal.a_max

    def test_input_bounds_ks(self):
        assert self.ks_dynamics.input_bounds.lb[0] == self.ks_dynamics.parameters.steering.v_min
        assert self.ks_dynamics.input_bounds.ub[0] == self.ks_dynamics.parameters.steering.v_max
        assert self.ks_dynamics.input_bounds.lb[1] == -self.ks_dynamics.parameters.longitudinal.a_max
        assert self.ks_dynamics.input_bounds.ub[1] == self.ks_dynamics.parameters.longitudinal.a_max

    def test_input_bounds_kst(self):
        assert self.kst_dynamics.input_bounds.lb[0] == self.kst_dynamics.parameters.steering.v_min
        assert self.kst_dynamics.input_bounds.ub[0] == self.kst_dynamics.parameters.steering.v_max
        assert self.kst_dynamics.input_bounds.lb[1] == -self.kst_dynamics.parameters.longitudinal.a_max
        assert self.kst_dynamics.input_bounds.ub[1] == self.kst_dynamics.parameters.longitudinal.a_max

    def test_input_bounds_st(self):
        assert self.st_dynamics.input_bounds.lb[0] == self.st_dynamics.parameters.steering.v_min
        assert self.st_dynamics.input_bounds.ub[0] == self.st_dynamics.parameters.steering.v_max
        assert self.st_dynamics.input_bounds.lb[1] == -self.st_dynamics.parameters.longitudinal.a_max
        assert self.st_dynamics.input_bounds.ub[1] == self.st_dynamics.parameters.longitudinal.a_max

    def test_input_bounds_mb(self):
        assert self.st_dynamics.input_bounds.lb[0] == self.st_dynamics.parameters.steering.v_min
        assert self.st_dynamics.input_bounds.ub[0] == self.st_dynamics.parameters.steering.v_max
        assert self.st_dynamics.input_bounds.lb[1] == -self.st_dynamics.parameters.longitudinal.a_max
        assert self.st_dynamics.input_bounds.ub[1] == self.st_dynamics.parameters.longitudinal.a_max

    def test_input_bounds_lks(self):
        assert self.lks_dynamics.input_bounds.lb[0] == -self.lks_dynamics.parameters.longitudinal.j_dot_max
        assert self.lks_dynamics.input_bounds.ub[0] == self.lks_dynamics.parameters.longitudinal.j_dot_max
        assert self.lks_dynamics.input_bounds.lb[1] == -self.lks_dynamics.parameters.steering.kappa_dot_dot_max
        assert self.lks_dynamics.input_bounds.ub[1] == self.lks_dynamics.parameters.steering.kappa_dot_dot_max

    def test_input_within_bounds_pm(self):
        max_input = PMInputState(
            acceleration=self.pm_dynamics.parameters.longitudinal.a_max + 0.001,
            acceleration_y=self.pm_dynamics.parameters.longitudinal.a_max + 0.001,
            time_step=0
        )

        assert not self.pm_dynamics.input_within_bounds(max_input)

    def test_input_within_bounds_ks(self):
        max_input = InputState(
            steering_angle_speed=self.ks_dynamics.parameters.steering.v_max + 0.001,
            acceleration=self.ks_dynamics.parameters.longitudinal.a_max + 0.001,
            time_step=0
        )

        assert not self.ks_dynamics.input_within_bounds(max_input)

    def test_input_within_bounds_kst(self):
        max_input = InputState(
            steering_angle_speed=self.kst_dynamics.parameters.steering.v_max + 0.001,
            acceleration=self.kst_dynamics.parameters.longitudinal.a_max + 0.001,
            time_step=0
        )

        assert not self.kst_dynamics.input_within_bounds(max_input)

    def test_input_within_bounds_st(self):
        max_input = InputState(
            steering_angle_speed=self.st_dynamics.parameters.steering.v_max + 0.001,
            acceleration=self.st_dynamics.parameters.longitudinal.a_max + 0.001,
            time_step=0
        )

        assert not self.st_dynamics.input_within_bounds(max_input)

    def test_input_within_bounds_mb(self):
        max_input = InputState(
            steering_angle_speed=self.mb_dynamics.parameters.steering.v_max + 0.001,
            acceleration=self.mb_dynamics.parameters.longitudinal.a_max + 0.001,
            time_step=0
        )

        assert not self.mb_dynamics.input_within_bounds(max_input)

    def test_input_within_bounds_lks(self):
        max_input = LKSInputState(
            jerk_dot=self.lks_dynamics.parameters.longitudinal.j_dot_max + 0.001,
            kappa_dot_dot=self.lks_dynamics.parameters.steering.kappa_dot_dot_max + 0.001,
            time_step=0
        )

        assert not self.lks_dynamics.input_within_bounds(max_input)

    def test_violates_friction_constraint_pm(self):
        max_input = PMInputState(
            acceleration=self.pm_dynamics.parameters.longitudinal.a_max,
            acceleration_y=self.pm_dynamics.parameters.longitudinal.a_max,
            time_step=0
        )
        # input to friction check -> acceleration x and acceleration y
        assert self.pm_dynamics.violates_friction_circle(self.random_pm_state, max_input)

    def test_violates_friction_constraint_ks(self):
        max_input = InputState(
            steering_angle_speed=0,
            acceleration=self.ks_dynamics.parameters.longitudinal.a_max,
            time_step=0
        )
        # input to friction check -> acceleration, velocity and steering angle
        self.zero_ks_init_state.velocity = self.ks_dynamics.parameters.longitudinal.v_max
        self.zero_ks_init_state.steering_angle = self.ks_dynamics.parameters.steering.max
        assert self.ks_dynamics.violates_friction_circle(self.zero_ks_init_state, max_input)

    def test_violates_friction_constraint_kst(self):
        max_input = InputState(
            steering_angle_speed=0,
            acceleration=self.kst_dynamics.parameters.longitudinal.a_max,
            time_step=0
        )
        # input to friction check -> acceleration, velocity and steering angle
        self.zero_kst_init_state.velocity = self.kst_dynamics.parameters.longitudinal.v_max
        self.zero_kst_init_state.steering_angle = self.kst_dynamics.parameters.steering.max
        assert self.kst_dynamics.violates_friction_circle(self.zero_kst_init_state, max_input)

    def test_violates_friction_constraint_st(self):
        max_input = InputState(
            steering_angle_speed=0,
            acceleration=self.st_dynamics.parameters.longitudinal.a_max,
            time_step=0
        )
        # input to friction check -> acceleration, velocity and yaw_rate
        self.zero_st_init_state.velocity = self.st_dynamics.parameters.longitudinal.v_max
        self.zero_st_init_state.yaw_rate = self.st_dynamics.parameters.steering.v_max
        assert self.st_dynamics.violates_friction_circle(self.zero_st_init_state, max_input)

    def test_violates_friction_constraint_mb(self):
        max_input = InputState(
            steering_angle_speed=self.mb_dynamics.parameters.steering.v_max,
            acceleration=self.mb_dynamics.parameters.longitudinal.a_max,
            time_step=0
        )
        # input to friction check -> acceleration, velocity and yaw_rate
        self.zero_mb_init_state.velocity = self.mb_dynamics.parameters.longitudinal.v_max
        self.zero_mb_init_state.yaw_rate = self.mb_dynamics.parameters.steering.v_max
        assert self.mb_dynamics.violates_friction_circle(self.zero_mb_init_state, max_input)

    def _test_for_inputs(self, vehicle, inputs, init_state):
        x, x_ts = vehicle.state_to_array(init_state)
        for inp in inputs:
            u, u_ts = vehicle.input_to_array(inp)

            if type(vehicle) is not LinearizedKSDynamics:
                if vehicle.violates_friction_circle(x, u):
                    continue

            expected_x1 = odeint(vehicle.dynamics, x, [0.0, self.dt],
                                 args=(u,), tfirst=True)[1]
            x1 = vehicle.forward_simulation(x, u, self.dt)

            for idx in range(len(x1)):
                assert x1[idx] == expected_x1[idx]

    def test_forward_simulation_pm(self):
        if self.disable_pm_tests:
            return
        self._test_for_inputs(self.pm_dynamics, self.pm_inputs, self.zero_pm_init_state)

    def test_forward_simulation_pm_velocity_bounds(self):
        if self.disable_pm_tests:
            return
        state = self.zero_pm_init_state
        state.velocity = self.pm_dynamics.parameters.longitudinal.v_max
        state.velocity_y = self.pm_dynamics.parameters.longitudinal.v_max

        self._test_for_inputs(self.pm_dynamics, self.pm_inputs, state)

    def test_forward_simulation_ks(self):
        if self.disable_ks_tests:
            return
        self._test_for_inputs(self.ks_dynamics, self.inputs, self.zero_ks_init_state)

    def test_forward_simulation_ks_velocity_and_steering_bounds(self):
        if self.disable_ks_tests:
            return
        state = self.zero_ks_init_state
        state.velocity = self.ks_dynamics.parameters.longitudinal.v_max
        state.steering_angle = self.ks_dynamics.parameters.steering.max

        self._test_for_inputs(self.ks_dynamics, self.inputs, state)

    def test_forward_simulation_ks_velocity_switch_point(self):
        if self.disable_ks_tests:
            return
        state = self.zero_ks_init_state
        state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch

        self._test_for_inputs(self.ks_dynamics, self.inputs, state)

    def test_forward_simulation_ks_velocity_below_switch_point(self):
        if self.disable_ks_tests:
            return
        state = self.zero_ks_init_state
        state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch - 0.001

        self._test_for_inputs(self.ks_dynamics, self.inputs, state)

    def test_forward_simulation_ks_velocity_above_switch_point(self):
        if self.disable_ks_tests:
            return
        state = self.zero_ks_init_state
        state.velocity = self.ks_dynamics.parameters.longitudinal.v_switch + 0.001

        self._test_for_inputs(self.ks_dynamics, self.inputs, state)

    def test_forward_simulation_kst(self):
        if self.disable_kst_tests:
            return
        self._test_for_inputs(self.kst_dynamics, self.inputs, self.zero_kst_init_state)

    def test_forward_simulation_kst_velocity_and_steering_bounds(self):
        if self.disable_kst_tests:
            return
        state = self.zero_kst_init_state
        state.velocity = self.kst_dynamics.parameters.longitudinal.v_max
        state.steering_angle = self.kst_dynamics.parameters.steering.max

        self._test_for_inputs(self.kst_dynamics, self.inputs, state)

    def test_forward_simulation_kst_velocity_switch_point(self):
        if self.disable_kst_tests:
            return
        state = self.zero_kst_init_state
        state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch

        self._test_for_inputs(self.kst_dynamics, self.inputs, state)

    def test_forward_simulation_kst_velocity_below_switch_point(self):
        if self.disable_kst_tests:
            return
        state = self.zero_kst_init_state
        state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch - 0.001

        self._test_for_inputs(self.kst_dynamics, self.inputs, state)

    def test_forward_simulation_kst_velocity_above_switch_point(self):
        if self.disable_kst_tests:
            return
        state = self.zero_kst_init_state
        state.velocity = self.kst_dynamics.parameters.longitudinal.v_switch + 0.001

        self._test_for_inputs(self.kst_dynamics, self.inputs, state)

    def test_forward_simulation_st(self):
        if self.disable_st_tests:
            return
        self._test_for_inputs(self.st_dynamics, self.inputs, self.zero_st_init_state)

    def test_forward_simulation_st_velocity_and_steering_bounds(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = self.st_dynamics.parameters.longitudinal.v_max
        state.steering_angle = self.st_dynamics.parameters.steering.max

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_st_velocity_switch_point(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = self.st_dynamics.parameters.longitudinal.v_switch

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_st_velocity_below_switch_point(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = self.st_dynamics.parameters.longitudinal.v_switch - 0.001

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_st_velocity_above_switch_point(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = self.st_dynamics.parameters.longitudinal.v_switch + 0.001

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_st_low_velocity_switch_point(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = 0.1

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_st_low_velocity_switch_point_below(self):
        if self.disable_st_tests:
            return
        state = self.zero_st_init_state
        state.velocity = 0.09

        self._test_for_inputs(self.st_dynamics, self.inputs, state)

    def test_forward_simulation_mb(self):
        if self.disable_mb_tests:
            return
        self._test_for_inputs(self.mb_dynamics, self.inputs, self.zero_mb_init_state)

    def test_forward_simulation_mb_velocity_and_steering_bounds(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = self.mb_dynamics.parameters.longitudinal.v_max
        state.steering_angle = self.mb_dynamics.parameters.steering.max

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_mb_velocity_switch_point(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_mb_velocity_below_switch_point(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch - 0.001

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_mb_velocity_above_switch_point(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = self.mb_dynamics.parameters.longitudinal.v_switch + 0.001

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_mb_low_velocity_switch_point(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = 0.1

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_mb_low_velocity_switch_point_below(self):
        if self.disable_mb_tests:
            return
        state = self.zero_mb_init_state
        state.velocity = 0.09

        self._test_for_inputs(self.mb_dynamics, self.inputs, state)

    def test_forward_simulation_lks(self):
        if self.disable_lks_tests:
            return
        self._test_for_inputs(self.lks_dynamics, self.lks_inputs, self.zero_lks_init_state)

    def test_forward_simulation_pm_sanity_check(self):
        if self.disable_pm_tests:
            return
        inp = PMInputState(acceleration=10.0, acceleration_y=0.0, time_step=0)
        x, x_ts = self.pm_dynamics.state_to_array(self.zero_pm_init_state)
        u, u_ts = self.pm_dynamics.input_to_array(inp)

        sim_state = self.pm_dynamics.forward_simulation(x, u, self.dt)

        self.assertAlmostEqual(sim_state[0], 0.05)  # x position
        self.assertAlmostEqual(sim_state[1], 0.0)  # y position
        self.assertAlmostEqual(sim_state[2], 1.0)  # velocity x
        self.assertAlmostEqual(sim_state[3], 0.0)  # velocity y

    def test_forward_simulation_ks_sanity_check(self):
        if self.disable_ks_tests:
            return
        inp = InputState(acceleration=10.0, steering_angle_speed=0.0, time_step=0)
        x, x_ts = self.ks_dynamics.state_to_array(self.zero_ks_init_state)
        u, u_ts = self.ks_dynamics.input_to_array(inp)

        sim_state_array = self.ks_dynamics.forward_simulation(x, u, self.dt)

        sim_state = self.ks_dynamics.array_to_state(sim_state_array, time_step=1)

        self.assertAlmostEqual(sim_state.position[0], 0.05)  # x position
        self.assertAlmostEqual(sim_state.position[1], 0.0)  # y position
        self.assertEqual(sim_state.steering_angle, 0.0)  # steering_angle
        self.assertEqual(sim_state.velocity, 1)  # velocity
        self.assertEqual(sim_state.orientation, 0.0)  # orientation

    def test_forward_simulation_kst_sanity_check(self):
        if self.disable_kst_tests:
            return
        inp = InputState(acceleration=10.0, steering_angle_speed=0.0, time_step=0)
        x, x_ts = self.kst_dynamics.state_to_array(self.zero_ks_init_state)
        u, u_ts = self.kst_dynamics.input_to_array(inp)

        sim_state = self.kst_dynamics.forward_simulation(x, u, self.dt)

        self.assertAlmostEqual(sim_state[0], 0.05)  # x position
        self.assertAlmostEqual(sim_state[1], 0.0)  # y position
        self.assertEqual(sim_state[2], 0.0)  # steering_angle
        self.assertEqual(sim_state[3], 1)  # velocity
        self.assertEqual(sim_state[4], 0.0)  # orientation
        self.assertEqual(sim_state[5], 0.0)  # hitch_angle

    def test_forward_simulation_st_sanity_check(self):
        if self.disable_st_tests:
            return
        inp = InputState(acceleration=10.0, steering_angle_speed=0.0, time_step=0)
        x, x_ts = self.st_dynamics.state_to_array(self.zero_st_init_state)
        u, u_ts = self.st_dynamics.input_to_array(inp)

        sim_state = self.st_dynamics.forward_simulation(x, u, self.dt)

        self.assertAlmostEqual(sim_state[0], 0.05)  # x position
        self.assertAlmostEqual(sim_state[1], 0.0)  # y position
        self.assertEqual(sim_state[2], 0.0)  # steering_angle
        self.assertEqual(sim_state[3], 1)  # velocity
        self.assertEqual(sim_state[4], 0.0)  # orientation
        self.assertEqual(sim_state[5], 0.0)  # yaw_rate
        self.assertEqual(sim_state[6], 0.0)  # slip_angle

    def test_forward_simulation_mb_sanity_check(self):
        if self.disable_mb_tests:
            return
        inp = InputState(acceleration=10.0, steering_angle_speed=0.0, time_step=0)
        x, x_ts = self.mb_dynamics.state_to_array(self.zero_st_init_state)
        u, u_ts = self.mb_dynamics.input_to_array(inp)

        sim_state = self.mb_dynamics.forward_simulation(x, u, self.dt)

        self.assertAlmostEqual(sim_state[0], 0.05)  # x position
        self.assertAlmostEqual(sim_state[1], 0.0)  # y position
        self.assertAlmostEqual(sim_state[2], 0.0)  # steering_angle
        self.assertAlmostEqual(sim_state[3], 0.1)  # velocity
        self.assertAlmostEqual(sim_state[4], 0.0)  # orientation
        self.assertAlmostEqual(sim_state[5], 0.0)  # yaw_rate
        self.assertAlmostEqual(sim_state[6], 0.0)  # roll_angle
        self.assertAlmostEqual(sim_state[7], 0.0)  # roll_rate
        self.assertAlmostEqual(sim_state[8], 0.0)  # pitch_angle
        self.assertAlmostEqual(sim_state[9], 0.0)  # pitch_rate
        self.assertAlmostEqual(sim_state[10], 0.0)  # velocity_y
        self.assertAlmostEqual(sim_state[11], 0.0)  # position_z
        self.assertAlmostEqual(sim_state[12], 0.0)  # velocity_z
        self.assertAlmostEqual(sim_state[13], 0.0)  # roll_angle_front
        self.assertAlmostEqual(sim_state[14], 0.0)  # roll_rate_front
        self.assertAlmostEqual(sim_state[15], 0.0)  # velocity_y_front
        self.assertAlmostEqual(sim_state[16], 0.0)  # position_z_front
        self.assertAlmostEqual(sim_state[17], 0.0)  # velocity_z_front
        self.assertAlmostEqual(sim_state[18], 0.0)  # roll_angle_rear
        self.assertAlmostEqual(sim_state[19], 0.0)  # roll_rate_rear
        self.assertAlmostEqual(sim_state[20], 0.0)  # velocity_y_rear
        self.assertAlmostEqual(sim_state[21], 0.0)  # position_z_rear
        self.assertAlmostEqual(sim_state[22], 0.0)  # velocity_z_rear
        self.assertAlmostEqual(sim_state[23], 0.0)  # left_front_wheel_angular_speed
        self.assertAlmostEqual(sim_state[24], 0.0)  # right_front_wheel_angular_speed
        self.assertAlmostEqual(sim_state[25], 0.0)  # left_rear_wheel_angular_speed
        self.assertAlmostEqual(sim_state[26], 0.0)  # right_rear_wheel_angular_speed
        self.assertAlmostEqual(sim_state[27], 0.0)  # delta_y_f
        self.assertAlmostEqual(sim_state[28], 0.0)  # delta_y_r

    def test_simulate_next_state_pm(self):
        if self.disable_pm_tests:
            return

        while self.pm_dynamics.violates_friction_circle(self.zero_pm_init_state, self.random_pm_input):
            self.random_pm_input = DummyDataGenerator.create_random_pm_input()

        x, x_ts = self.pm_dynamics.state_to_array(self.zero_pm_init_state)
        u, u_ts = self.pm_dynamics.input_to_array(self.random_pm_input)

        x1 = odeint(self.pm_dynamics.dynamics, x, [0.0, self.dt], args=(u,), tfirst=True)[1]
        next_state = self.pm_dynamics.simulate_next_state(self.zero_pm_init_state, self.random_pm_input, self.dt)

        assert x1[0] == next_state.position[0]
        assert x1[1] == next_state.position[1]
        assert x1[2] == next_state.velocity
        assert x1[3] == next_state.velocity_y
        assert x_ts + 1 == next_state.time_step

    def test_simulate_next_state_ks(self):
        if self.disable_ks_tests:
            return

        while self.ks_dynamics.violates_friction_circle(self.zero_ks_init_state, self.random_ks_input):
            self.random_ks_input = DummyDataGenerator.create_random_input()

        x, x_ts = self.ks_dynamics.state_to_array(self.zero_ks_init_state)
        u, u_ts = self.ks_dynamics.input_to_array(self.random_ks_input)

        x1 = odeint(self.ks_dynamics.dynamics, x, [0.0, self.dt], args=(u,), tfirst=True)[1]
        x1_state = self.ks_dynamics.array_to_state(x1, time_step=1)

        next_state = self.ks_dynamics.simulate_next_state(self.zero_ks_init_state, self.random_ks_input, self.dt)

        assert x1_state.position[0] == next_state.position[0]
        assert x1_state.position[1] == next_state.position[1]
        assert x1_state.steering_angle == next_state.steering_angle
        assert x1_state.velocity == next_state.velocity
        assert x1_state.orientation == next_state.orientation
        assert x_ts + 1 == next_state.time_step

    def test_simulate_next_state_kst(self):
        if self.disable_kst_tests:
            return

        while self.kst_dynamics.violates_friction_circle(self.zero_kst_init_state, self.random_kst_input):
            self.random_kst_input = DummyDataGenerator.create_random_input()

        x, x_ts = self.kst_dynamics.state_to_array(self.zero_kst_init_state)
        u, u_ts = self.kst_dynamics.input_to_array(self.random_kst_input)

        x1 = odeint(self.kst_dynamics.dynamics, x, [0.0, self.dt], args=(u,), tfirst=True)[1]
        next_state = self.kst_dynamics.simulate_next_state(self.zero_ks_init_state, self.random_kst_input, self.dt)

        assert x1[0] == next_state.position[0]
        assert x1[1] == next_state.position[1]
        assert x1[2] == next_state.steering_angle
        assert x1[3] == next_state.velocity
        assert x1[4] == next_state.orientation
        assert x1[5] == next_state.hitch_angle
        assert x_ts + 1 == next_state.time_step

    def test_simulate_next_state_st(self):
        if self.disable_st_tests:
            return

        while self.st_dynamics.violates_friction_circle(self.zero_st_init_state, self.random_st_input):
            self.random_st_input = DummyDataGenerator.create_random_input()

        x, x_ts = self.st_dynamics.state_to_array(self.zero_st_init_state)
        u, u_ts = self.st_dynamics.input_to_array(self.random_st_input)

        x1 = odeint(self.st_dynamics.dynamics, x, [0.0, self.dt], args=(u,), tfirst=True)[1]
        next_state = self.st_dynamics.simulate_next_state(self.zero_st_init_state, self.random_st_input, self.dt)

        assert x1[0] == next_state.position[0]
        assert x1[1] == next_state.position[1]
        assert x1[2] == next_state.steering_angle
        assert x1[3] == next_state.velocity
        assert x1[4] == next_state.orientation
        assert x1[5] == next_state.yaw_rate
        assert x1[6] == next_state.slip_angle
        assert x_ts + 1 == next_state.time_step

    def test_simulate_next_state_mb(self):
        if self.disable_mb_tests:
            return

        while self.mb_dynamics.violates_friction_circle(self.zero_mb_init_state, self.random_input):
            self.random_input = DummyDataGenerator.create_random_input()

        x, x_ts = self.mb_dynamics.state_to_array(self.zero_mb_init_state)
        u, u_ts = self.mb_dynamics.input_to_array(self.random_input)

        x1 = odeint(self.mb_dynamics.dynamics, x, [0.0, self.dt], args=(u, self.mb_dynamics.parameters), tfirst=True)[1]
        next_state = self.mb_dynamics.simulate_next_state(self.zero_mb_init_state, self.random_input, self.dt)

        assert x1[0] == next_state.position[0]
        assert x1[1] == next_state.position[1]
        assert x1[2] == next_state.steering_angle
        assert x1[3] == next_state.velocity
        assert x1[4] == next_state.orientation
        assert x1[5] == next_state.yaw_rate
        assert x1[6] == next_state.roll_angle
        assert x1[7] == next_state.roll_rate
        assert x1[8] == next_state.pitch_angle
        assert x1[9] == next_state.pitch_rate
        assert x1[10] == next_state.velocity_y
        assert x1[11] == next_state.position_z
        assert x1[12] == next_state.velocity_z
        assert x1[13] == next_state.roll_angle_front
        assert x1[14] == next_state.roll_rate_front
        assert x1[15] == next_state.velocity_y_front
        assert x1[16] == next_state.position_z_front
        assert x1[17] == next_state.velocity_z_front
        assert x1[18] == next_state.roll_angle_rear
        assert x1[19] == next_state.roll_rate_rear
        assert x1[20] == next_state.velocity_y_rear
        assert x1[21] == next_state.position_z_rear
        assert x1[22] == next_state.velocity_z_rear
        assert x1[23] == next_state.left_front_wheel_angular_speed
        assert x1[24] == next_state.right_front_wheel_angular_speed
        assert x1[25] == next_state.left_rear_wheel_angular_speed
        assert x1[26] == next_state.right_rear_wheel_angular_speed
        assert x1[27] == next_state.delta_y_f
        assert x1[28] == next_state.delta_y_r
        assert x_ts + 1 == next_state.time_step

    def test_simulate_next_state_lks(self):
        if self.disable_lks_tests:
            return

        # TODO: friction circle ?
        """while self.pm_dynamics.violates_friction_circle(self.zero_pm_init_state, self.random_pm_input):
            self.random_pm_input = DummyDataGenerator.create_random_pm_input()"""

        x, x_ts = self.lks_dynamics.state_to_array(self.zero_lks_init_state)
        u, u_ts = self.lks_dynamics.input_to_array(self.random_lks_input)

        x1 = odeint(self.lks_dynamics.dynamics, x, [0.0, self.dt], args=(u,), tfirst=True)[1]
        next_state = self.lks_dynamics.simulate_next_state(self.zero_lks_init_state, self.random_lks_input, self.dt)

        next_lon_state = next_state[0]
        next_lat_state = next_state[1]
        assert x1[0] == next_lon_state.longitudinal_position
        assert x1[1] == next_lon_state.velocity
        assert x1[2] == next_lon_state.acceleration
        assert x1[3] == next_lon_state.jerk
        assert x1[4] == next_lat_state.lateral_position
        assert x1[5] == next_lat_state.orientation
        assert x1[6] == next_lat_state.curvature
        assert x1[7] == next_lat_state.curvature_rate
        assert x_ts + 1 == next_lon_state.time_step

    def _simulate_trajectory(self, vehicle, init_state, inp_generator):
        """ Try to create a random valid trajectory """
        idx_try = 0
        violates = True
        valid_states, valid_inps = None, None
        while violates:
            states = [init_state]
            inps = []
            traj_invalid = False
            for idx in range(5):
                inp = inp_generator(time_step=idx)
                if vehicle.violates_friction_circle(states[-1], inp):
                    idx_try += 1
                    traj_invalid = True
                    break
                next_state = vehicle.simulate_next_state(states[-1], inp, self.dt, throw=False)
                states.append(next_state)
                inps.append(inp)
            if not traj_invalid:
                violates = False
                valid_states, valid_inps = states, inps
        trajectory = Trajectory(initial_time_step=0, state_list=valid_states)
        input_vector = Trajectory(initial_time_step=0, state_list=valid_inps)
        return trajectory, input_vector

    def test_simulate_trajectory_pm(self):
        if self.disable_pm_tests:
            return
        expected_trajectory, input_vector = self._simulate_trajectory(self.pm_dynamics, self.random_pm_init_state,
                                                                      DummyDataGenerator.create_random_pm_input)

        simulated_trajectory = self.pm_dynamics.simulate_trajectory(self.random_pm_init_state, input_vector, self.dt)

        for state, expected_state in zip(simulated_trajectory.state_list, expected_trajectory.state_list):
            assert state.position[0] == expected_state.position[0]
            assert state.position[1] == expected_state.position[1]
            assert state.velocity == expected_state.velocity
            assert state.velocity_y == expected_state.velocity_y

    def test_simulate_trajectory_ks(self):
        if self.disable_ks_tests:
            return
        expected_trajectory, input_vector = self._simulate_trajectory(self.ks_dynamics, self.random_ks_init_state,
                                                                      DummyDataGenerator.create_random_input)

        simulated_trajectory = self.ks_dynamics.simulate_trajectory(self.random_ks_init_state, input_vector, self.dt)

        for state, expected_state in zip(simulated_trajectory.state_list, expected_trajectory.state_list):
            assert state.position[0] == expected_state.position[0]
            assert state.position[1] == expected_state.position[1]
            assert state.steering_angle == expected_state.steering_angle
            assert state.velocity == expected_state.velocity
            assert state.orientation == expected_state.orientation

    def test_simulate_trajectory_kst(self):
        if self.disable_kst_tests:
            return
        expected_trajectory, input_vector = self._simulate_trajectory(self.kst_dynamics, self.random_kst_init_state,
                                                                      DummyDataGenerator.create_random_input)

        simulated_trajectory = self.kst_dynamics.simulate_trajectory(self.random_kst_init_state, input_vector, self.dt)

        for state, expected_state in zip(simulated_trajectory.state_list, expected_trajectory.state_list):
            assert state.position[0] == expected_state.position[0]
            assert state.position[1] == expected_state.position[1]
            assert state.steering_angle == expected_state.steering_angle
            assert state.velocity == expected_state.velocity
            assert state.orientation == expected_state.orientation
            assert state.hitch_angle == expected_state.hitch_angle

    def test_simulate_trajectory_st(self):
        if self.disable_st_tests:
            return
        expected_trajectory, input_vector = self._simulate_trajectory(self.st_dynamics, self.random_st_init_state,
                                                                      DummyDataGenerator.create_random_input)

        simulated_trajectory = self.st_dynamics.simulate_trajectory(self.random_st_init_state, input_vector, self.dt)

        for state, expected_state in zip(simulated_trajectory.state_list, expected_trajectory.state_list):
            assert state.position[0] == expected_state.position[0]
            assert state.position[1] == expected_state.position[1]
            assert state.steering_angle == expected_state.steering_angle
            assert state.velocity == expected_state.velocity
            assert state.orientation == expected_state.orientation
            assert state.yaw_rate == expected_state.yaw_rate
            assert state.slip_angle == expected_state.slip_angle

    def test_simulate_trajectory_mb(self):
        if self.disable_mb_tests:
            return
        expected_trajectory, input_vector = self._simulate_trajectory(self.mb_dynamics, self.random_mb_init_state,
                                                                      DummyDataGenerator.create_random_input)

        simulated_trajectory = self.mb_dynamics.simulate_trajectory(self.random_mb_init_state, input_vector, self.dt)

        for state, expected_state in zip(simulated_trajectory.state_list, expected_trajectory.state_list):
            assert state.position[0] == expected_state.position[0]
            assert state.position[1] == expected_state.position[1]
            assert state.steering_angle == expected_state.steering_angle
            assert state.velocity == expected_state.velocity
            assert state.orientation == expected_state.orientation
            assert state.yaw_rate == expected_state.yaw_rate
            assert state.roll_angle == expected_state.roll_angle
            assert state.roll_rate == expected_state.roll_rate
            assert state.pitch_angle == expected_state.pitch_angle
            assert state.pitch_rate == expected_state.pitch_rate
            assert state.velocity_y == expected_state.velocity_y
            assert state.position_z == expected_state.position_z
            assert state.velocity_z == expected_state.velocity_z
            assert state.roll_angle_front == expected_state.roll_angle_front
            assert state.roll_rate_front == expected_state.roll_rate_front
            assert state.velocity_y_front == expected_state.velocity_y_front
            assert state.position_z_front == expected_state.position_z_front
            assert state.velocity_z_front == expected_state.velocity_z_front
            assert state.roll_angle_rear == expected_state.roll_angle_rear
            assert state.roll_rate_rear == expected_state.roll_rate_rear
            assert state.velocity_y_rear == expected_state.velocity_y_rear
            assert state.position_z_rear == expected_state.position_z_rear
            assert state.velocity_z_rear == expected_state.velocity_z_rear
            assert state.left_front_wheel_angular_speed == expected_state.left_front_wheel_angular_speed
            assert state.right_front_wheel_angular_speed == expected_state.right_front_wheel_angular_speed
            assert state.left_rear_wheel_angular_speed == expected_state.left_rear_wheel_angular_speed
            assert state.right_rear_wheel_angular_speed == expected_state.right_rear_wheel_angular_speed
            assert state.delta_y_f == expected_state.delta_y_f
            assert state.delta_y_r == expected_state.delta_y_r


if __name__ == '__main__':
    unittest.main()
