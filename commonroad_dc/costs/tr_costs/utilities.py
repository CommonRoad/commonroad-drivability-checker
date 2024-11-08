
from pathlib import Path
from ruamel.yaml import YAML
from vehiclemodels import parameters_vehicle2

from crmonitor.common.world import RoadNetwork, Vehicle, CurvilinearStateManager
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory
from commonroad.geometry.shape import Rectangle




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
    ego_trajectory : Trajectory, road_network: RoadNetwork, config
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
    
    # Create a lanelet assignment for the ego vehicle , mapping maybe corrected
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
            acceleration = (state.velocity - prev_state.velocity) / 0.1  # scenario.dt , using 0.1 for now 
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
        0,
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
