from abc import ABC, abstractmethod
from enum import Enum, unique
from typing import List, Union, Tuple

import numpy as np
import math
from commonroad.common.solution import VehicleType, VehicleModel
from commonroad.common.util import make_valid_orientation
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, InputState, PMInputState, PMState, KSState, KSTState, STState, \
    MBState, LongitudinalState, LateralState, LKSInputState
from scipy.integrate import odeint
from scipy.optimize import Bounds
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
from vehiclemodels.parameters_vehicle4 import parameters_vehicle4
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_kst import vehicle_dynamics_kst
from vehiclemodels.vehicle_dynamics_linearized import vehicle_dynamics_linearized
from vehiclemodels.vehicle_parameters import VehicleParameters


# supported vehicle model state classes in the VehicleDynamics classes of the feasibility checker
VehicleModelStates = Union[PMState, KSState, KSTState, STState, MBState]

# supported input state classes in the VehicleDynamics classes of the feasibility checker
InputStateClasses = Union[InputState, PMInputState, LKSInputState]


class VehicleDynamicsException(Exception):
    pass


class FrictionCircleException(VehicleDynamicsException):
    pass


class InputBoundsException(VehicleDynamicsException):
    pass


class StateException(VehicleDynamicsException):
    pass


class InputException(VehicleDynamicsException):
    pass


@unique
class VehicleParameterMapping(Enum):
    """
    Mapping for VehicleType name to VehicleParameters
    """
    FORD_ESCORT = parameters_vehicle1()
    BMW_320i = parameters_vehicle2()
    VW_VANAGON = parameters_vehicle3()
    TRUCK = parameters_vehicle4()

    @classmethod
    def from_vehicle_type(cls, vehicle_type: VehicleType) -> VehicleParameters:
        return cls[vehicle_type.name].value


class VehicleDynamics(ABC):
    """
    VehicleDynamics abstract class that encapsulates the common methods of all VehicleDynamics classes.

    List of currently implemented vehicle models
     - Point-Mass Model (PM)
     - Kinematic Single-Track Model (KS)
     - Linearized Kinematic Single-Track Model (LKS)
     - Kinematic Single-Track Trailer Model (KST)
     - Single-Track Model (ST)
     - Multi-Body Model (MB)

    New types of VehicleDynamics can be defined by extending this class. If there isn't any mismatch with the state
    values, the new VehicleDynamics class can be used directly with the feasibility checkers as well.

    For detailed documentation of the Vehicle Models, please check the `Vehicle Model Documentation
    <https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf>`_
    """

    def __init__(self, vehicle_model: VehicleModel, vehicle_type: VehicleType):
        """
        Creates a VehicleDynamics model for the given VehicleType.

        :param vehicle_type: VehicleType
        """
        self.vehicle_model = vehicle_model
        self.vehicle_type = vehicle_type
        self.parameters = VehicleParameterMapping[self.vehicle_type.name].value
        self.shape = Rectangle(length=self.parameters.l, width=self.parameters.w)

    @classmethod
    def PM(cls, vehicle_type: VehicleType) -> 'PointMassDynamics':
        """
        Creates a PointMassDynamics model.

        :param vehicle_type: VehicleType, i.e. VehileType.FORD_ESCORT
        :return: PointMassDynamics instance with the given vehicle type.
        """
        return PointMassDynamics(vehicle_type)

    @classmethod
    def KS(cls, vehicle_type: VehicleType) -> 'KinematicSingleTrackDynamics':
        """
        Creates a KinematicSingleTrackDynamics model.

        :param vehicle_type: VehicleType, i.e. VehileType.FORD_ESCORT
        :return: KinematicSingleTrackDynamics instance with the given vehicle type.
        """
        return KinematicSingleTrackDynamics(vehicle_type)

    @classmethod
    def ST(cls, vehicle_type: VehicleType) -> 'SingleTrackDynamics':
        """
        Creates a SingleTrackDynamics VehicleDynamics model.

        :param vehicle_type: VehicleType, i.e. VehileType.FORD_ESCORT
        :return: SingleTrackDynamics instance with the given vehicle type.
        """
        return SingleTrackDynamics(vehicle_type)

    @classmethod
    def MB(cls, vehicle_type: VehicleType) -> 'MultiBodyDynamics':
        """
        Creates a MultiBodyDynamics VehicleDynamics model.

        :param vehicle_type: VehicleType, i.e. VehileType.FORD_ESCORT
        :return: MultiBodyDynamics instance with the given vehicle type.
        """
        return MultiBodyDynamics(vehicle_type)

    @classmethod
    def KST(cls, vehicle_type: VehicleType) -> 'KinematicSingleTrackTrailerDynamics':
        """
        Creates a KinematicSingleTrackTrailerDynamics VehicleDynamics model.

        :param vehicle_type: VehicleType, i.e. VehicleType.FORD_ESCORT
        :return: KinematicSingleTrackTrailerDynamics instance with the given vehicle type.
        """
        return KinematicSingleTrackTrailerDynamics(vehicle_type)

    @classmethod
    def LKS(cls, vehicle_type: VehicleType, ref_pos: np.ndarray, ref_theta: np.ndarray) -> 'LinearizedKSDynamics':
        """
        Creates a LinearizedKSDynamics VehicleDynamics model.

        :param vehicle_type: VehicleType, i.e. VehicleType.FORD_ESCORT
        :param ref_pos: longitudinal position (s) of each vertex of the reference path
        :param ref_theta: orientations (theta) at each vertex of the reference path
        :return: KinematicSingleTrackTrailerDynamics instance with the given vehicle type.
        """
        return LinearizedKSDynamics(vehicle_type, ref_pos=ref_pos, ref_theta=ref_theta)

    @classmethod
    def from_model(cls, vehicle_model: VehicleModel, vehicle_type: VehicleType) -> 'VehicleDynamics':
        """
        Creates a VehicleDynamics model for the given vehicle model and type.

        :param vehicle_model: VehicleModel, i.e. VehicleModel.KS
        :param vehicle_type: VehicleType, i.e. VehileType.FORD_ESCORT
        :return: VehicleDynamics instance with the given vehicle type.
        """
        model_constructor = getattr(cls, vehicle_model.name)
        return model_constructor(vehicle_type)

    @abstractmethod
    def dynamics(self, t, x, u) -> List[float]:
        """
        Vehicle dynamics function that models the motion of the vehicle during forward simulation.

        :param t: time point which the differentiation is being calculated at.
        :param x: state values
        :param u: input values
        :return: next state values
        """
        pass

    @property
    def input_bounds(self) -> Bounds:
        """
        Returns the bounds on inputs (constraints).

        Bounds are
            - min steering velocity <= steering_angle_speed <= max steering velocity
            - -max longitudinal acc <= acceleration <= max longitudinal acc

        :return: Bounds
        """
        return Bounds([self.parameters.steering.v_min, -self.parameters.longitudinal.a_max],
                      [self.parameters.steering.v_max, self.parameters.longitudinal.a_max])

    def input_within_bounds(self, u: Union[InputStateClasses, np.array], throw: bool = False) -> bool:
        """
        Checks whether the given input is within input constraints of the vehicle dynamics model.

        :param u: input values as np.array or State - Contains 2 values
        :param throw: if set to false, will return bool instead of throwing exception (default=False)
        :return: True if within constraints
        """
        inputs = self.input_to_array(u)[0] if isinstance(u, InputStateClasses.__args__) else u
        in_bounds = all([self.input_bounds.lb[idx] <= round(inputs[idx], 4) <= self.input_bounds.ub[idx]
                         for idx in range(len(self.input_bounds.lb))])
        if not in_bounds and throw:
            raise InputBoundsException(f'Input is not within bounds!\nInput: {u}')
        return in_bounds

    def violates_friction_circle(self, x: Union[VehicleModelStates, np.array], u: Union[InputState, np.array],
                                 throw: bool = False) -> bool:
        """
        Checks whether given input violates the friction circle constraint for the given state.

        :param x: current state
        :param u: the input which was used to simulate the next state
        :param throw: if set to false, will return bool instead of throwing exception (default=False)
        :return: True if the constraint was violated
        """
        x_vals = self.state_to_array(x)[0] if isinstance(x, VehicleModelStates.__args__) else x
        u_vals = self.input_to_array(u)[0] if isinstance(u, InputState) else u
        x_dot = self.dynamics(0, x_vals, u_vals)

        vals = np.array([u_vals[1], x_vals[3] * x_dot[4]])
        vals_sq = np.power(vals, 2)
        vals_sum = np.sum(vals_sq)
        violates = vals_sum > self.parameters.longitudinal.a_max ** 2

        if throw and violates:
            msg = f'Input violates friction circle constraint!\n' \
                  f'Init state: {x}\n\n Input:{u}'
            raise FrictionCircleException(msg)

        return violates

    def forward_simulation(self, x: np.array, u: np.array, dt: float, throw: bool = True) -> Union[None, np.array]:
        """
        Simulates the next state using the given state and input values as numpy arrays.

        :param x: state values.
        :param u: input values
        :param dt: scenario delta time.
        :param throw: if set to false, will return None as next state instead of throwing exception (default=True)
        :return: simulated next state values, raises VehicleDynamicsException if invalid input.
        """
        within_bounds = self.input_within_bounds(u, throw)
        violates_friction_constraint = self.violates_friction_circle(x, u, throw)
        if not throw and (not within_bounds or violates_friction_constraint):
            return None

        x0, x1 = odeint(self.dynamics, x, [0.0, dt], args=(u,), tfirst=True)
        return x1

    def simulate_next_state(self, x: Union[VehicleModelStates, Tuple[LongitudinalState, LateralState]],
                            u: InputStateClasses, dt: float, throw: bool = True) -> \
            Union[None, VehicleModelStates, Tuple[LongitudinalState, LateralState]]:
        """
        Simulates the next state using the given state and input values as State objects.

        :param x: current state
        :param u: inputs for simulating the next state
        :param dt: scenario delta time.
        :param throw: if set to false, will return None as next state instead of throwing exception (default=True)
        :return: simulated next state, raises VehicleDynamicsException if invalid input.
        """
        x_vals, x_ts = self.state_to_array(x)
        u_vals, u_ts = self.input_to_array(u)
        x1_vals = self.forward_simulation(x_vals, u_vals, dt, throw)
        if x1_vals is None:
            return None
        x1 = self.array_to_state(x1_vals, x_ts + 1)
        return x1

    def simulate_trajectory(self, initial_state: InitialState, input_vector: Trajectory,
                            dt: float, throw: bool = True) -> Union[None, Trajectory]:
        """
        Creates the trajectory for the given input vector.

        :param initial_state: initial state of the planning problem
        :param input_vector: input vector as Trajectory object
        :param dt: scenario delta time
        :param throw: if set to false, will return None as trajectory instead of throwing exception (default=True)
        :return: simulated trajectory, raises VehicleDynamicsException if there is an invalid input.
        """
        converted_init_state = self.convert_initial_state(initial_state)
        state_list = [converted_init_state]
        for input in input_vector.state_list:
            simulated_state = self.simulate_next_state(state_list[-1], input, dt, throw)
            if not throw and not simulated_state:
                return None
            state_list.append(simulated_state)
        trajectory = Trajectory(initial_time_step=initial_state.time_step, state_list=state_list)
        return trajectory

    @abstractmethod
    def _state_to_array(self, state: Union[VehicleModelStates, InitialState], steering_angle_default=0.0) \
            -> Tuple[np.array, int]:
        """Actual conversion of state to array happens here, each vehicle will implement its own converter."""
        pass

    def state_to_array(self, state: Union[VehicleModelStates, InitialState], steering_angle_default=0.0) \
            -> Tuple[np.array, int]:
        """
        Converts the given State to numpy array.

        :param state: State
        :return: state values as numpy array and time step of the state
        """
        try:
            array, time_step = self._state_to_array(state, steering_angle_default)
            return array, time_step
        except Exception as e:
            err = f'Not a valid state!\nState:{str(state)}'
            raise StateException(err) from e

    @abstractmethod
    def _array_to_state(self, x: np.array, time_step: int) -> VehicleModelStates:
        """Actual conversion of the array to state happens here, each vehicle will implement its own converter."""
        pass

    def array_to_state(self, x: np.array, time_step: int) -> VehicleModelStates:
        """
        Converts the given numpy array of values to State.

        :param x: list of state values
        :param time_step: time step of the converted state
        :return: State
        """
        try:
            state = self._array_to_state(x, time_step)
            return state
        except Exception as e:
            err = f'Not a valid state array!\nTime step: {time_step}, State array:{str(x)}'
            raise StateException(err) from e

    def convert_initial_state(self, initial_state: InitialState, steering_angle_default=0.0) -> VehicleModelStates:
        """
        Converts the given default initial state to VehicleModel's state by setting the state values accordingly.

        :param initial_state: default initial state
        :param steering_angle_default: default steering_angle value as it is not given in intiial state
        :return: converted initial state
        """
        return self.array_to_state(self.state_to_array(initial_state, steering_angle_default)[0],
                                   initial_state.time_step)

    def _input_to_array(self, input: InputStateClasses) -> Tuple[np.array, int]:
        """
        Actual conversion of input to array happens here. Vehicles can override this method to implement their own
        converter.
        """
        time_step = input.time_step
        return np.array(input), time_step

    def input_to_array(self, input: InputStateClasses) -> Tuple[np.array, int]:
        """
        Converts the given input (as State object) to numpy array.

        :param input: input as State object
        :return: state values as numpy array and time step of the state, raises VehicleDynamicsException if invalid
            input
        """
        try:
            array, time_step = self._input_to_array(input)
            return array, time_step
        except Exception as e:
            raise InputException(f'Not a valid input!\n{str(input)}') from e

    def _array_to_input(self, u: np.array, time_step: int) -> InputState:
        """
        Actual conversion of input array to input happens here. Vehicles can override this method to implement their
        own converter.
        """
        values = {
            'steering_angle_speed': u[0],
            'acceleration': u[1],
        }
        return InputState(**values, time_step=time_step)

    def array_to_input(self, u: np.array, time_step: int) -> InputStateClasses:
        """
        Converts the given numpy array of values to input (as State object).

        :param u: input values
        :param time_step: time step of the converted input
        :return: input as state object, raises VehicleDynamicsException if invalid input
        """
        try:
            state = self._array_to_input(u, time_step)
            return state
        except Exception as e:
            raise InputException(f'Not a valid input array!\nArray:{str(u)} Time Step: {time_step}') from e

    @staticmethod
    def _convert_from_directional_velocity(velocity, orientation) -> Tuple[float, float]:
        """
        Converts the given velocity and orientation to velocity_x and velocity_y values.

        :param velocity: velocity
        :param orientation: orientation
        :return: velocity_x, velocity_y
        """
        velocity_x = math.cos(orientation) * velocity
        velocity_y = math.sin(orientation) * velocity
        return velocity_x, velocity_y


class PointMassDynamics(VehicleDynamics):

    def __init__(self, vehicle_type: VehicleType):
        super(PointMassDynamics, self).__init__(VehicleModel.PM, vehicle_type)

    def dynamics(self, t, x, u) -> List[float]:
        """
        Point Mass model dynamics function. Overrides the dynamics function of VehicleDynamics for PointMass model.

        :param t:
        :param x: state values, [position x, position y, velocity x, velocity y]
        :param u: input values, [acceleration x, acceleration y]

        :return:
        """
        return [
            x[2],
            x[3],
            u[0],
            u[1],
        ]

    @property
    def input_bounds(self) -> Bounds:
        """
        Overrides the bounds method of Vehicle Model in order to return bounds for the Point Mass inputs.

        Bounds are
            - -max longitudinal acc <= acceleration <= max longitudinal acc
            - -max longitudinal acc <= acceleration_y <= max longitudinal acc

        :return: Bounds
        """
        return Bounds([-self.parameters.longitudinal.a_max, -self.parameters.longitudinal.a_max],
                      [self.parameters.longitudinal.a_max, self.parameters.longitudinal.a_max])

    def violates_friction_circle(self, x: Union[PMState, np.array], u: Union[PMInputState, np.array],
                                 throw: bool = False) -> bool:
        """
        Overrides the friction circle constraint method of Vehicle Model in order calculate
        friction circle constraint for the Point Mass model.

        :param x: current state
        :param u: the input which was used to simulate the next state
        :param throw: if set to false, will return bool instead of throwing exception (default=False)
        :return: True if the constraint was violated
        """
        u_vals = self.input_to_array(u)[0] if isinstance(u, PMInputState) else u

        vals_sq = np.power(u_vals, 2)
        vals_sqrt = np.sqrt(np.sum(vals_sq))
        violates = vals_sqrt > self.parameters.longitudinal.a_max
        if throw and violates:
            msg = f'Input violates friction circle constraint!\n' \
                  f'Init state: {x}\n\n Input:{u}'
            raise FrictionCircleException(msg)

        return violates

    def _state_to_array(self, state: Union[PMState, InitialState], steering_angle_default=0.0) -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        time_step = state.time_step

        if hasattr(state, 'velocity') and hasattr(state, 'orientation') and not \
                hasattr(state, 'velocity_y'):  # If initial state
            velocity_x, velocity_y = self._convert_from_directional_velocity(state.velocity, state.orientation)
            equivalent_pmstate = PMState(time_step=time_step, position=state.position,
                                         velocity=velocity_x, velocity_y=velocity_y)
            return np.array(equivalent_pmstate), time_step
        else:
            return np.array(state), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> PMState:
        """ Implementation of the VehicleDynamics abstract method. """
        values = {
            'position': np.array([x[0], x[1]]),
            'velocity': x[2],
            'velocity_y': x[3]
        }
        return PMState(**values, time_step=time_step)

    def _array_to_input(self, u: np.array, time_step: int) -> PMInputState:
        """ Overrides VehicleDynamics method. """
        values = {
            'acceleration': u[0],
            'acceleration_y': u[1],
        }
        return PMInputState(**values, time_step=time_step)


class KinematicSingleTrackDynamics(VehicleDynamics):
    def __init__(self, vehicle_type: VehicleType):
        super(KinematicSingleTrackDynamics, self).__init__(VehicleModel.KS, vehicle_type)

    def dynamics(self, t, x, u) -> List[float]:
        return vehicle_dynamics_ks(x, u, self.parameters)

    def _state_to_array(self, state: Union[KSState, InitialState], steering_angle_default=0.0) -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        values = [
            state.position[0] - self.parameters.b * math.cos(state.orientation),
            state.position[1] - self.parameters.b * math.sin(state.orientation),
            getattr(state, 'steering_angle', steering_angle_default),  # not defined in initial state
            state.velocity,
            state.orientation
        ]
        time_step = state.time_step
        return np.array(values), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> KSState:
        """ Implementation of the VehicleDynamics abstract method. """
        values = {
            'position': np.array([x[0] + self.parameters.b * math.cos(x[4]),
                                  x[1] + self.parameters.b * math.sin(x[4])]),
            'steering_angle': x[2],
            'velocity': x[3],
            'orientation': x[4],
        }
        state = KSState(**values, time_step=time_step)
        return state


class SingleTrackDynamics(VehicleDynamics):
    def __init__(self, vehicle_type: VehicleType):
        super(SingleTrackDynamics, self).__init__(VehicleModel.ST, vehicle_type)

    def dynamics(self, t, x, u) -> List[float]:
        return vehicle_dynamics_st(x, u, self.parameters)

    def _state_to_array(self, state: Union[STState, InitialState], steering_angle_default=0.0) -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        values = [
            state.position[0],
            state.position[1],
            getattr(state, 'steering_angle', steering_angle_default),  # not defined in initial state
            state.velocity,
            state.orientation,
            state.yaw_rate,
            state.slip_angle
        ]
        time_step = state.time_step
        return np.array(values), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> STState:
        """ Implementation of the VehicleDynamics abstract method. """
        values = {
            'position': np.array([x[0], x[1]]),
            'steering_angle': x[2],
            'velocity': x[3],
            'orientation': x[4],
            'yaw_rate': x[5],
            'slip_angle': x[6],
        }
        return STState(**values, time_step=time_step)


class KinematicSingleTrackTrailerDynamics(VehicleDynamics):
    def __init__(self, vehicle_type: VehicleType):
        assert vehicle_type == VehicleType.TRUCK, "The KinematicSingleTrackTrailerDynamics is only supported for " \
                                                  "the truck model (VehicleType.TRUCK)"
        super(KinematicSingleTrackTrailerDynamics, self).__init__(VehicleModel.KST, vehicle_type)

    def dynamics(self, t, x, u) -> List[float]:
        return vehicle_dynamics_kst(x, u, self.parameters)

    def convert_initial_state(self, initial_state: InitialState, steering_angle_default=0.0, hitch_angle_default=0.0) \
            -> KSTState:
        """
        Overrides method convert_initial_state() in VehicleDynamics Base class due to additional hitch angle
        Converts the given default initial state to VehicleModel's state by setting the state values accordingly.

        :param initial_state: default initial state
        :param steering_angle_default: default steering_angle value as it is not given in initial state
        :param hitch_angle_default: default hitch angle as it is not given in initial state
        :return: converted initial state
        """
        return self.array_to_state(self.state_to_array(initial_state, steering_angle_default, hitch_angle_default)[0],
                                   initial_state.time_step)

    def state_to_array(self, state: Union[KSTState, InitialState], steering_angle_default=0.0, hitch_angle_default=0.0) \
            -> Tuple[np.array, int]:
        """
        Overrides method state_to_array() from VehicleDynamics Base class due to additional hitch angle
        Converts the given State to numpy array.

        :param state: State
        :param steering_angle_default: default steering_angle value as it is not given in initial state
        :param hitch_angle_default: default hitch angle as it is not given in initial state
        :return: state values as numpy array and time step of the state
        """
        try:
            array, time_step = self._state_to_array(state, steering_angle_default, hitch_angle_default)
            return array, time_step
        except Exception as e:
            err = f'Not a valid state!\nState:{str(state)}'
            raise StateException(err) from e

    def _state_to_array(self, state: Union[KSTState, InitialState], steering_angle_default=0.0, hitch_angle_default=0.0) -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        values = [
            state.position[0],
            state.position[1],
            getattr(state, 'steering_angle', steering_angle_default),  # not defined in initial state
            state.velocity,
            state.orientation,
            getattr(state, 'hitch_angle', hitch_angle_default) # not defined in initial state
        ]
        time_step = state.time_step
        return np.array(values), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> KSTState:
        """ Implementation of the VehicleDynamics abstract method. """
        values = {
            'position': np.array([x[0], x[1]]),
            'steering_angle': x[2],
            'velocity': x[3],
            'orientation': x[4],
            'hitch_angle': x[5]
        }
        state = KSTState(**values, time_step=time_step)
        return state


class MultiBodyDynamics(VehicleDynamics):
    def __init__(self, vehicle_type: VehicleType):
        super(MultiBodyDynamics, self).__init__(VehicleModel.MB, vehicle_type)

    def dynamics(self, t, x, u) -> List[float]:
        return vehicle_dynamics_mb(x, u, self.parameters)

    def _state_to_array(self, state: Union[MBState, InitialState], steering_angle_default=0.0) -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        if not len(state.attributes) == 29:  # if initial state
            velocity_x, velocity_y = self._convert_from_directional_velocity(state.velocity, state.orientation)
        else:
            velocity_x, velocity_y = state.velocity, state.velocity_y

        p = self.parameters
        g = 9.81  # [m/s^2]
        F0_z_f = p.m_s * g * p.b / (p.a + p.b) + p.m_uf * g
        F0_z_r = p.m_s * g * p.a / (p.a + p.b) + p.m_ur * g
        position_z_front = F0_z_f / 2 * p.K_zt
        position_z_rear = F0_z_r / 2 * p.K_zt

        def wheel_speed(vel_x):
            return vel_x / p.R_w

        def velocity_y_front(vel_y, yaw_rate):
            return vel_y + p.a * yaw_rate

        def velocity_y_rear(vel_y, yaw_rate):
            return vel_y - p.b * yaw_rate

        values = [
            # sprung mass states
            state.position[0],
            state.position[1],
            getattr(state, 'steering_angle', steering_angle_default),  # not defined in initial state
            velocity_x,
            state.orientation,
            state.yaw_rate,
            getattr(state, 'roll_angle', 0.0),
            getattr(state, 'roll_rate', 0.0),
            getattr(state, 'pitch_angle', 0.0),
            getattr(state, 'pitch_rate', 0.0),
            getattr(state, 'velocity_y', velocity_y),
            getattr(state, 'position_z', 0.0),
            getattr(state, 'velocity_z', 0.0),

            # unsprung mass states (front)
            getattr(state, 'roll_angle_front', 0.0),
            getattr(state, 'roll_rate_front', 0.0),
            getattr(state, 'velocity_y_front', velocity_y_front(velocity_y, state.yaw_rate)),
            getattr(state, 'position_z_front', position_z_front),
            getattr(state, 'velocity_z_front', 0.0),  # not defined in initial state

            # unsprung mass states (rear)
            getattr(state, 'roll_angle_rear', 0.0),  # not defined in initial state
            getattr(state, 'roll_rate_rear', 0.0),  # not defined in initial state
            getattr(state, 'velocity_y_rear', velocity_y_rear(velocity_y, state.yaw_rate)),
            getattr(state, 'position_z_rear', position_z_rear),
            getattr(state, 'velocity_z_rear', 0.0),  # not defined in initial state

            # wheel states
            getattr(state, 'left_front_wheel_angular_speed', wheel_speed(velocity_x)),
            getattr(state, 'right_front_wheel_angular_speed', wheel_speed(velocity_x)),
            getattr(state, 'left_rear_wheel_angular_speed', wheel_speed(velocity_x)),
            getattr(state, 'right_rear_wheel_angular_speed', wheel_speed(velocity_x)),
            getattr(state, 'delta_y_f', 0.0),  # not defined in initial state
            getattr(state, 'delta_y_r', 0.0),  # not defined in initial state
        ]
        time_step = state.time_step
        return np.array(values), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> MBState:
        """ Implementation of the VehicleDynamics abstract method. """
        values = {
            'position': np.array([x[0], x[1]]),
            'steering_angle': x[2],
            'velocity': x[3],
            'orientation': x[4],
            'yaw_rate': x[5],
            'roll_angle': x[6],
            'roll_rate': x[7],
            'pitch_angle': x[8],
            'pitch_rate': x[9],
            'velocity_y': x[10],
            'position_z': x[11],
            'velocity_z': x[12],
            'roll_angle_front': x[13],
            'roll_rate_front': x[14],
            'velocity_y_front': x[15],
            'position_z_front': x[16],
            'velocity_z_front': x[17],
            'roll_angle_rear': x[18],
            'roll_rate_rear': x[19],
            'velocity_y_rear': x[20],
            'position_z_rear': x[21],
            'velocity_z_rear': x[22],
            'left_front_wheel_angular_speed': x[23],
            'right_front_wheel_angular_speed': x[24],
            'left_rear_wheel_angular_speed': x[25],
            'right_rear_wheel_angular_speed': x[26],
            'delta_y_f': x[27],
            'delta_y_r': x[28],
        }
        return MBState(**values, time_step=time_step)


class LinearizedKSDynamics(VehicleDynamics):
    def __init__(self, vehicle_type: VehicleType, ref_pos: np.ndarray, ref_theta: np.ndarray):
        super(LinearizedKSDynamics, self).__init__(VehicleModel.MB, vehicle_type)
        self.ref_pos = ref_pos
        self.ref_theta = ref_theta

    def dynamics(self, t, x, u) -> List[float]:
        """Implementation of the VehicleDynamics abstract method"""
        return vehicle_dynamics_linearized(x, u, self.parameters, self.ref_pos, self.ref_theta)

    @property
    def input_bounds(self) -> Bounds:
        """
        Overrides the input_bounds() method of VehicleDynamics Base class in order to return bounds for the Linearized
        KS inputs.

        Bounds are
            - max jerk_dot <= jerk_dot <= max jerk_dot
            - min kappa_dot_dot <= kappa_dot_dot <= max kappa_dot_dot

        :return: Bounds
        """
        return Bounds([-self.parameters.longitudinal.j_dot_max, -self.parameters.steering.kappa_dot_dot_max],
                      [self.parameters.longitudinal.j_dot_max, self.parameters.steering.kappa_dot_dot_max])

    def forward_simulation(self, x: np.array, u: np.array, dt: float, throw: bool = True) -> Union[None, np.array]:
        """
        Overrides method forward_simulation() from VehicleDynamics Base class!
        Simulates the next state using the given state and input values as numpy arrays.

        :param x: state values.
        :param u: input values
        :param dt: scenario delta time.
        :param throw: if set to false, will return None as next state instead of throwing exception (default=True)
        :return: simulated next state values, raises VehicleDynamicsException if invalid input.
        """
        within_bounds = self.input_within_bounds(u, throw)
        if not throw and not within_bounds:
            return None

        x0, x1 = odeint(self.dynamics, x, [0.0, dt], args=(u,), tfirst=True)
        return x1

    def _state_to_array(self, state: Tuple[LongitudinalState, LateralState], steering_angle_default=0.0) \
            -> Tuple[np.array, int]:
        """ Implementation of the VehicleDynamics abstract method. """
        lon_state = state[0]
        lat_state = state[1]
        assert lon_state.time_step == lat_state.time_step, "Time steps of longitudinal and lateral state do not match."
        values = [
            lon_state.longitudinal_position,
            lon_state.velocity,
            lon_state.acceleration,
            lon_state.jerk,
            lat_state.lateral_position,
            lat_state.orientation,
            lat_state.curvature,
            lat_state.curvature_rate
        ]
        time_step = lon_state.time_step
        return np.array(values), time_step

    def _array_to_state(self, x: np.array, time_step: int) -> Tuple[LongitudinalState, LateralState]:
        """ Implementation of the VehicleDynamics abstract method. """
        values_lon = {
            'longitudinal_position': x[0],
            'velocity': x[1],
            'acceleration': x[2],
            'jerk': x[3]
        }
        values_lat = {
            'lateral_position': x[4],
            'orientation': make_valid_orientation(x[5]),
            'curvature': x[6],
            'curvature_rate': x[7]
        }
        return LongitudinalState(**values_lon, time_step=time_step), LateralState(**values_lat, time_step=time_step)

    def _array_to_input(self, u: np.array, time_step: int) -> LKSInputState:
        """Overrides method _array_to_input() from VehicleDynamics Base class."""
        values = {
            'jerk_dot': u[0],
            'kappa_dot_dot': u[1],
        }
        return LKSInputState(**values, time_step=time_step)

    def convert_initial_state(self, initial_state: InitialState, steering_angle_default=0.0) -> None:
        """Overrides method _array_to_input() from VehicleDynamics Base class."""

        msg = "Conversion of InitialState is not supported for the LinearizedKSDynamics Model. Please convert the" \
              "initial state to LongitudinalState and LateralState beforehand"
        raise NotImplementedError(msg)

    def violates_friction_circle(self, x: Union[Tuple[LongitudinalState, LateralState], np.array],
                                 u: Union[LKSInputState, np.array], throw: bool = False) -> bool:
        """
        Overrides method violates_friction_circle() from VehicleDynamics Base class!
        Checks whether the given state violates the friction circle constraint for the LKS model

        :param x: current state
        :param u: the input which was used to simulate the next state
        :param throw: if set to false, will return bool instead of throwing exception (default=False)
        :return: True if the constraint was violated
        """
        x_vals = self.state_to_array(x)[0] if isinstance(x, tuple) and \
                                              list(map(type, x)) == [LongitudinalState, LateralState] else x

        # Friction Circle Formula: a_long^2 + (v^2 * kappa)^2
        vals = np.array([x_vals[2], x_vals[1]*x_vals[1]*x_vals[6]])
        vals_sq = np.power(vals, 2)
        vals_sum = np.sum(vals_sq)

        violates = vals_sum > self.parameters.longitudinal.a_max ** 2

        if throw and violates:
            msg = f'Input violates friction circle constraint!\n' \
                  f'Init state: {x}\n\n Input:{u}'
            raise FrictionCircleException(msg)

        return violates

    def violates_state_constraints(self, x: Union[Tuple[LongitudinalState, LateralState], np.array]) -> bool:
        """
        Checks whether the given state violates state constraints for the LKS model
        :param x: current state
        :return: True if one constraint is violated
        """
        x_vals = self.state_to_array(x)[0] if isinstance(x, tuple) and \
                                              list(map(type, x)) == [LongitudinalState, LateralState] else x

        # velocity
        if x_vals[1] < self.parameters.longitudinal.v_min or x_vals > self.parameters.longitudinal.v_max:
            return True

        # acceleration
        if np.abs(x_vals[2]) > self.parameters.longitudinal.a_max:
            return True

        # jerk
        if np.abs(x_vals[3]) > self.parameters.longitudinal.j_max:
            return True

        # curvature
        l_wb = self.parameters.a + self.parameters.b
        kappa_max = np.tan(self.parameters.steering.max) / l_wb
        kappa_min = np.tan(self.parameters.steering.min) / l_wb
        if x_vals[6] > kappa_max or x_vals[6] < kappa_min:
            return True

        # curvature rate
        if np.abs(x_vals[7]) > self.parameters.steering.kappa_dot_max:
            return True

        return False
