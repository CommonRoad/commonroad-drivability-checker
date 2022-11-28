import math
import numpy as np
import random
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, PMState, KSState, KSTState, STState, MBState, InputState, \
    PMInputState, LKSInputState, LongitudinalState, LateralState


class DummyDataGenerator:

    @staticmethod
    def create_random_float_uniform(lower, upper) -> float:
        return random.uniform(lower, upper)

    @staticmethod
    def create_random_float_normal(lower, upper) -> float:
        random_val = np.random.normal((upper + lower) / 2, (upper - lower) / 6, 1)[0]
        return min(upper, max(lower, random_val))

    @classmethod
    def create_random_float(cls, lower, upper, dist='uniform') -> float:
        if dist == 'uniform':
            return cls.create_random_float_uniform(lower, upper)
        elif dist == 'normal':
            return cls.create_random_float_normal(lower, upper)
        else:
            return cls.create_random_float_uniform(lower, upper)

    @classmethod
    def create_random_initial_state(cls, pos_min=0.0, pos_max=0.0, v_min=-11.2, v_max=41.7, yaw_rate_max=0.4):
        return InitialState(
            position=np.array([cls.create_random_float(pos_min, pos_max),
                               cls.create_random_float(pos_min, pos_max)]),
            velocity=cls.create_random_float(v_min, v_max),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            acceleration=0.0,
            yaw_rate=cls.create_random_float(-yaw_rate_max, yaw_rate_max),
            slip_angle=cls.create_random_float(-np.math.pi / 12, np.math.pi / 12),
            time_step=0
        )

    @classmethod
    def create_random_initial_state_lks(cls,v_min=0.0, v_max=41.7, a_max=11.0, j_max=10.0e3, theta_max=0.2,
                                        kappa_max=0.6, kappa_dot_max=0.4):
        """For the linearized kinematic single track model (LKS) we directly  create a LongitudinalState and
        LateralState as initial states, as the conversion from the CommonRoad InitialState would require a
        curvilinear coordinate system transformation"""
        lon_state = LongitudinalState(longitudinal_position=0.0,
                                      velocity=cls.create_random_float(v_min, v_max),
                                      acceleration=cls.create_random_float(-a_max, a_max),
                                      jerk=cls.create_random_float(-j_max, j_max),
                                      time_step=0
                                      )
        lat_state = LateralState(lateral_position=0.0,
                                 orientation=cls.create_random_float(-theta_max, theta_max),
                                 curvature=cls.create_random_float(-kappa_max, kappa_max),
                                 curvature_rate=cls.create_random_float(-kappa_dot_max, kappa_dot_max),
                                 time_step=0
                                 )
        return lon_state, lat_state

    @classmethod
    def create_zero_initial_state(cls):
        return InitialState(
            position=np.array([0.0, 0.0]),
            velocity=0.0,
            orientation=0.0,
            acceleration=0.0,
            yaw_rate=0.0,
            slip_angle=0.0,
            time_step=0
        )

    @classmethod
    def create_zero_initial_state_lks(cls):
        """For the linearized kinematic single track model (LKS) we directly  create a LongitudinalState and
        LateralState as initial states, as the conversion from the CommonRoad InitialState would require a
        curvilinear coordinate system transformation"""
        lon_state = LongitudinalState(
            longitudinal_position=0.0,
            velocity=0.0,
            acceleration=0.0,
            jerk=0.0,
            time_step=0
        )

        lat_state = LateralState(
            lateral_position=0.0,
            orientation=0.0,
            curvature=0.0,
            curvature_rate=0.0,
            time_step=0
        )
        return lon_state, lat_state

    @classmethod
    def create_initial_state(cls, velocity):
        return InitialState(
            position=np.array([0.0, 0.0]),
            velocity=velocity,
            orientation=0.0,
            acceleration=0.0,
            yaw_rate=0.0,
            slip_angle=0.0,
            time_step=0
        )

    @classmethod
    def create_random_pm_state(cls, v_min=-11.2, v_max=41.7, time_step=0):
        return PMState(
            position=np.array([0,
                               0]),
            velocity=cls.create_random_float(v_min, v_max),
            velocity_y=cls.create_random_float(v_min, v_max),
            time_step=time_step
        )

    @classmethod
    def create_random_ks_state(cls, v_min=-11.2, v_max=41.7, steering_angle_max=0.910, time_step=0):
        return KSState(
            position=np.array([0,
                               0]),
            steering_angle=cls.create_random_float(-steering_angle_max, steering_angle_max),
            velocity=cls.create_random_float(v_min, v_max),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            time_step=time_step
        )

    @classmethod
    def create_random_kst_state(cls, v_min=-2.78, v_max=22.22, steering_angle_max=0.55, time_step=0):
        return KSTState(
            position=np.array([0,
                               0]),
            steering_angle=cls.create_random_float(-steering_angle_max, steering_angle_max),
            velocity=cls.create_random_float(v_min, v_max),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            time_step=time_step,
            hitch_angle=cls.create_random_float(-0.5, 0.5)
        )

    @classmethod
    def create_random_st_state(cls, v_min=-11.2, v_max=41.7, steering_angle_max=0.910, yaw_rate_max=0.4, time_step=0):
        return STState(
            position=np.array([0,
                               0]),
            steering_angle=cls.create_random_float(-steering_angle_max, steering_angle_max),
            velocity=cls.create_random_float(v_min, v_max),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            yaw_rate=cls.create_random_float(-yaw_rate_max, yaw_rate_max),
            slip_angle=cls.create_random_float(-np.math.pi / 12, np.math.pi / 12),
            time_step=time_step
        )

    @classmethod
    def create_random_mb_state(cls, vehicle_parameters, v_min=-11.2, v_max=41.7, steering_angle_max=0.910,
                               yaw_rate_max=0.4, time_step=0):
        velocity = cls.create_random_float(v_min, v_max)
        orientation = cls.create_random_float(-np.math.pi, np.math.pi)
        yaw_rate = cls.create_random_float(-yaw_rate_max, yaw_rate_max)
        velocity_x = math.cos(orientation) * velocity
        velocity_y = math.sin(orientation) * velocity
        velocity_y_front = velocity_y + vehicle_parameters.a * yaw_rate
        velocity_y_rear = velocity_y - vehicle_parameters.b * yaw_rate
        p = vehicle_parameters
        g = 9.81  # [m/s^2]
        F0_z_f = p.m_s * g * p.b / (p.a + p.b) + p.m_uf * g
        F0_z_r = p.m_s * g * p.a / (p.a + p.b) + p.m_ur * g
        position_z_front = F0_z_f / 2 * p.K_zt
        position_z_rear = F0_z_r / 2 * p.K_zt
        wheel_speed = velocity_x / vehicle_parameters.R_w
        return MBState(
            position=np.array([0,
                               0]),
            steering_angle=cls.create_random_float(-steering_angle_max, steering_angle_max),
            velocity=velocity_x,
            orientation=orientation,
            yaw_rate=yaw_rate,
            roll_angle=cls.create_random_float(-math.pi / 24, math.pi / 24),
            roll_rate=cls.create_random_float(-math.pi / 48, math.pi / 48),  # made up - half of the roll angle ???
            pitch_angle=cls.create_random_float(-math.pi / 24, math.pi / 24),
            pitch_rate=cls.create_random_float(-math.pi / 48, math.pi / 48),
            velocity_y=velocity_y,
            position_z=cls.create_random_float(0.5, 1),
            velocity_z=cls.create_random_float(-1, 1),
            roll_angle_front=cls.create_random_float(-math.pi / 24, math.pi / 24),
            roll_rate_front=cls.create_random_float(-math.pi / 48, math.pi / 48),
            velocity_y_front=velocity_y_front,
            position_z_front=position_z_front,
            velocity_z_front=cls.create_random_float(-1, 1),
            roll_angle_rear=cls.create_random_float(-math.pi / 24, math.pi / 24),
            roll_rate_rear=cls.create_random_float(-math.pi / 48, math.pi / 48),
            velocity_y_rear=velocity_y_rear,
            position_z_rear=position_z_rear,
            velocity_z_rear=cls.create_random_float(-1, 1),
            left_front_wheel_angular_speed=wheel_speed,
            right_front_wheel_angular_speed=wheel_speed,
            left_rear_wheel_angular_speed=wheel_speed,
            right_rear_wheel_angular_speed=wheel_speed,
            delta_y_f=cls.create_random_float(-1, 1),
            delta_y_r=cls.create_random_float(-1, 1),
            time_step=time_step
        )

    @classmethod
    def create_random_lks_state(cls, v_min=0.0, v_max=41.7, a_max=11.0, j_max=10.0e3, theta_max=0.2, kappa_max=0.6,
                                kappa_dot_max=0.4):
        lon_state = LongitudinalState(longitudinal_position=0.0,
                                      velocity=cls.create_random_float(v_min, v_max),
                                      acceleration=cls.create_random_float(-a_max, a_max),
                                      jerk=cls.create_random_float(-j_max, j_max),
                                      time_step=0
                                      )
        lat_state = LateralState(lateral_position=0.0,
                                 orientation=cls.create_random_float(-theta_max, theta_max),
                                 curvature=cls.create_random_float(-kappa_max, kappa_max),
                                 curvature_rate=cls.create_random_float(-kappa_dot_max, kappa_dot_max),
                                 time_step=0
                                 )
        return lon_state, lat_state

    @classmethod
    def create_random_input(cls, a_max=11.5, steering_angle_speed_max=0.4, time_step=0):
        return InputState(
            acceleration=cls.create_random_float(-a_max, a_max),
            steering_angle_speed=cls.create_random_float(-steering_angle_speed_max, steering_angle_speed_max),
            time_step=time_step
        )

    @classmethod
    def create_random_pm_input(cls, a_max=11.5, time_step=0):
        return PMInputState(
            acceleration=cls.create_random_float(-a_max, a_max),
            acceleration_y=cls.create_random_float(-a_max, a_max),
            time_step=time_step
        )

    @classmethod
    def create_random_lks_input(cls, j_dot_max=10.0e3, kappa_dot_dot_max = 20.0, time_step=0):
        return LKSInputState(
            jerk_dot=cls.create_random_float(-j_dot_max, j_dot_max),
            kappa_dot_dot=cls.create_random_float(-kappa_dot_dot_max, kappa_dot_dot_max),
            time_step=time_step
        )

    @staticmethod
    def _try_random_trajectory(init_state, vehicle, inp_generator, dt=0.1, state_count=5):
        states = [init_state]
        inps = []
        for idx in range(state_count):
            inp = inp_generator(time_step=idx)
            if vehicle.violates_friction_circle(states[-1], inp) or not vehicle.input_within_bounds(inp):
                return None, None
            next_state = vehicle.simulate_next_state(states[-1], inp, dt, throw=False)
            states.append(next_state)
            inps.append(inp)
        trajectory = Trajectory(initial_time_step=0, state_list=states)
        input_vector = Trajectory(initial_time_step=0, state_list=inps)
        return trajectory, input_vector

    @classmethod
    def _create_random_trajectory(cls, init_state, vehicle, inp_generator, dt=0.1, state_count=5):
        trajectory, input_vector = None, None
        while trajectory is None and input_vector is None:
            trajectory, input_vector = cls._try_random_trajectory(init_state, vehicle, inp_generator, dt, state_count)
        return trajectory, input_vector

    @classmethod
    def create_random_pm_trajectory(cls, init_state, vehicle, dt=0.1, state_count=5):
        return cls._create_random_trajectory(init_state, vehicle, cls.create_random_pm_input, dt, state_count)

    @classmethod
    def create_random_trajectory(cls, init_state, vehicle, dt=0.1, state_count=5):
        return cls._create_random_trajectory(init_state, vehicle, cls.create_random_input, dt, state_count)

    @classmethod
    def create_random_input_vector(cls, input_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_input(ts) for ts in range(input_count)])

    @classmethod
    def create_random_pm_input_vector(cls, input_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_pm_input(ts) for ts in range(input_count)])
