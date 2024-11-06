
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from crmonitor.common.world import World, RoadNetwork
from crmonitor.evaluation.evaluation import RuleEvaluator

from .utilities import *



class TrafficRuleCostEvaluator:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory, config_path: str = None):
        self._config = load_configuration(config_path)
        self._road_network = RoadNetwork(scenario.lanelet_network, self._config.get("road_network_param"))
        self._ego_vehicle = create_ego_vehicle_from_trajectory(
            ego_trajectory, self._road_network,  self._config
        )
        self._other_vehicles = [ create_other_vehicle_from_obstacle(
                                obstacle, self._road_network, self._config
                                )
                                for obstacle in scenario.dynamic_obstacles
        ]
        self._vehicles = [self._ego_vehicle] + self._other_vehicles
        self._world = World(set(self._vehicles), self._road_network, scenario)


    @property
    def world(self):
        return self._world

    @property
    def ego_vehicle_id(self):
        return self._ego_vehicle.id
    

    def evaluate_safe_distance(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G1"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_unnecessary_braking(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G2"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_maximum_speed_limit(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G3"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_traffic_flow(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G4"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_no_stopping(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I1"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_driving_faster_than_left_traffic(self) -> list[float]:
            rule_evaluator = RuleEvaluator.create_from_config(
                world=self.world, 
                ego_id=self.ego_vehicle_id, 
                rule="R_I2"
            )
            return rule_evaluator.evaluate()
        
    
    def evaluate_reversing_and_turning(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I3"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_emergency_lane(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I4"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_consider_entering_vehicle(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I5"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_stop_at_stop_sign(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN1_past"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_stop_at_traffic_light(self) -> list[float]:
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN2_TOR"
        )
        return rule_evaluator.evaluate()
    
