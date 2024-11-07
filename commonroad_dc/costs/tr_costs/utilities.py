
from crmonitor.common.world import RoadNetwork, Vehicle, CurvilinearStateManager
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory
from commonroad.geometry.shape import Rectangle
from vehiclemodels import parameters_vehicle2
from pathlib import Path
from ruamel.yaml import YAML



def load_configuration(config_path: str) -> dict:
    config = Path(config_path or Path(__file__).parent / "config.yaml")
    yaml_loader = YAML()
    with open(config, "r") as file:
        return yaml_loader.load(file)


def create_other_vehicle_from_obstacle(
    obstacle: DynamicObstacle, road_network: RoadNetwork, config
    ) -> Vehicle:
    vehicle_param = config.get("other_vehicles_param", {})
    lanelet_assignment = {
        state.time_step: set(
            road_network.lanelet_network.find_lanelet_by_position([state.position])[0]
        )
        for state in obstacle.prediction.trajectory.state_list
    }
    states_cr = {}
    for idx, state in enumerate(obstacle.prediction.trajectory.state_list):
        if idx > 0:
            prev_state = obstacle.prediction.trajectory.state_list[idx - 1]
            acceleration = (state.velocity - prev_state.velocity) / 0.1  # scenario.dt
        else:
            acceleration = 0 
        states_cr[state.time_step] = CustomState(
            position=state.position,
            velocity=state.velocity,
            orientation=state.orientation,
            time_step=state.time_step,
            acceleration=acceleration  
        )
    shape = obstacle.obstacle_shape
    vehicle = Vehicle(
        id=obstacle.obstacle_id,
        obstacle_type=obstacle.obstacle_type,
        vehicle_param=vehicle_param,
        shape=shape,
        states_cr=states_cr,
        signal_series=None,
        ccosy_cache=CurvilinearStateManager(road_network),
        lanelet_assignment=lanelet_assignment,
    )
    return vehicle

def create_ego_vehicle_from_trajectory(
    ego_trajectory : Trajectory, road_network: RoadNetwork, config
    ) -> Vehicle:
    ego_vehicle_param = config.get("ego_vehicle_param", {})
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