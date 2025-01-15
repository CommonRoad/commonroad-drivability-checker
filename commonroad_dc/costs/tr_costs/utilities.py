# utility functions and class

from pathlib import Path
from ruamel.yaml import YAML
from vehiclemodels import parameters_vehicle2

from crmonitor.common.world import RoadNetwork, Vehicle, CurvilinearStateManager
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory
from commonroad.geometry.shape import Rectangle
from crmonitor.predicates.scaling import RobustnessScalingConstants as constants

import numpy as np




def load_configuration(config_path: str) -> dict:
    """
    Load the configuration from a YAML file.

    Args:
        config_path (str): The path to the YAML configuration file.

    Returns:
        dict: The loaded configuration as a dictionary.
    """
    config = Path(config_path or Path(__file__).parent / "config.yaml")
    yaml_loader = YAML()
    with open(config, "r") as file:
        return yaml_loader.load(file)


def create_ego_vehicle_from_trajectory(
    ego_trajectory : Trajectory, road_network: RoadNetwork, config, dt
    ) -> Vehicle:
    """
    Creates an ego vehicle object from a given trajectory.

    Args:
        ego_trajectory (Trajectory): The trajectory of the ego vehicle.
        road_network (RoadNetwork): The road network object.
        config: Configuration parameters for the ego vehicle.

    Returns:
        Vehicle: The ego vehicle object.

    """
    ego_vehicle_param = config.get("ego_vehicle_param", {})
    
    # Create a lanelet assignment for the ego vehicle
    lanelet_assignment = {
        state.time_step: set(
            road_network.lanelet_network.find_lanelet_by_position([state.position])[0]
        )
        for state in ego_trajectory.state_list
    }
    
    states_cr = {}
    for idx, state in enumerate(ego_trajectory.state_list):
        if idx > 0:
            prev_state = ego_trajectory.state_list[idx - 1]
            acceleration = (state.velocity - prev_state.velocity) / dt 
        else:
            acceleration = 0
        
        states_cr[state.time_step] = CustomState(
            position=state.position,
            velocity=state.velocity,
            orientation=state.orientation,
            time_step=state.time_step,
            acceleration=acceleration 
        )
        
    ego_vehicle = Vehicle(
        -1,  # ego vehicle ID
        obstacle_type=ObstacleType.CAR,
        vehicle_param=ego_vehicle_param,
        shape=Rectangle(
            width=parameters_vehicle2.parameters_vehicle2().w, 
            length=parameters_vehicle2.parameters_vehicle2().l
        ),
        states_cr=states_cr,
        signal_series=None,
        ccosy_cache=CurvilinearStateManager(road_network),
        lanelet_assignment=lanelet_assignment,
    )
    
    return ego_vehicle


class NominalScaler:
    """
    For the duration-severity cost design, 
    we define a scaling factor. 
    This ensures that these partial cost functions assign a nominal cost J_{nominal} = 1.0
    for a specified nominal robustness value.
    We assume that the fine (weight) specified in BKatV for a rule violation corresponds to 
    the penalty incurred if the vehicle maintains a nominal robustness value 
    indicative of a critical violation throughout its trajectory.
    """

    def __init__(self):
        self.config = load_configuration(Path(__file__).parent.joinpath("config.yaml"))
        self.critical_value = self.config.get("nominal_unscaled_robustness")
        
    @classmethod
    def scale_distance_cost(cls, cost) -> float :
        upper_bound = constants.MAX_LONG_DIST
        unscaled_nominal_robustness = cls().critical_value["nominal_distance_rob"]
        nominalized_cost = (upper_bound/unscaled_nominal_robustness) * cost
        return nominalized_cost
    
    @classmethod
    def scale_speed_cost(cls, cost) -> float :
        upper_bound = constants.MAX_SPEED
        unscaled_nominal_robustness = cls().critical_value["nominal_speed_rob"]
        nominalized_cost = (upper_bound/unscaled_nominal_robustness) * cost
        return nominalized_cost
        
    @classmethod
    def scale_acceleration_cost(cls, cost) -> float :
        upper_bound = constants.MAX_ACC
        unscaled_nominal_robustness = cls().critical_value["nominal_acceleration_rob"]
        nominalized_cost = (upper_bound/unscaled_nominal_robustness) * cost
        return nominalized_cost 

