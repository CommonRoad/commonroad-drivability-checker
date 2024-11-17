
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from crmonitor.common.world import World, RoadNetwork
from crmonitor.evaluation.evaluation import RuleEvaluator
from .utilities import load_configuration, create_ego_vehicle_from_trajectory




class TrafficRuleCostEvaluator:
    def __init__(self, scenario: Scenario, 
                 ego_trajectory: Trajectory, 
                 config_path: str = None
                 ) -> None:
        """
        Initializes the TRCostEvaluator object.

        Args:
            scenario (Scenario): The scenario object representing the road network and other entities.
            ego_trajectory (Trajectory): The trajectory of the ego vehicle.
            config_path (str, optional): The path to the configuration file.
        """
        self._scenario = scenario
        self._ego_trajectory = ego_trajectory
        self._config = load_configuration(config_path)
        
        self._road_network = RoadNetwork(self._scenario.lanelet_network, 
                                         self._config.get("road_network_param", {}))
        
        self._world = World.create_from_scenario(self._scenario)
        
        self._ego_vehicle = create_ego_vehicle_from_trajectory(self._ego_trajectory, 
                                                               self._road_network, 
                                                               self._config)
        
        self._world.add_vehicle(self._ego_vehicle) # Add the ego vehicle to the world object.
        

    @property
    def world(self):
        return self._world


    @property
    def ego_vehicle_id(self):
        return self._ego_vehicle.id
    

    def evaluate_safe_distance(self) -> list[float]:
        """
        Evaluates the safe distance for the ego vehicle.

        Returns:
            A list of floats representing the safe distance robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G1"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_unnecessary_braking(self) -> list[float]:
        """
        Evaluates the cost of unnecessary braking for the ego vehicle.

        Returns:
            A list of floats representing the unnecessary braking robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G2"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_maximum_speed_limit(self) -> list[float]:
        """
        Evaluates the maximum speed limit for the ego vehicle.

        Returns:
            A list of floats representing the evaluated maximum speed limit for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G3"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_traffic_flow(self) -> list[float]:
        """
        Evaluates the traffic flow for the ego vehicle.
        
        Returns:
            A list of floats representing the traffic flow robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G4"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_no_stopping(self) -> list[float]:
        """
        Evaluates the no stopping rule for the ego vehicle.
        
        Returns:
            A list of floats representing the no stopping robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I1"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_driving_faster_than_left_traffic(self) -> list[float]:
        """ 
        Evaluates the rule of driving faster than the left traffic for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """    
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I2"
        )
        return rule_evaluator.evaluate()
        
    
    def evaluate_reversing_and_turning(self) -> list[float]:
        """
        Evaluates the rule of reversing and turning for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """   
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I3"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_emergency_lane(self) -> list[float]:
        """
        Evaluates the rule of emergency lane for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """    
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I4"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_consider_entering_vehicle(self) -> list[float]:
        """
        Evaluates the rule of considering entering vehicles for lane change for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I5"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_stop_at_stop_sign(self) -> list[float]:
        """
        Evaluates the rule of stopping at stop sign for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN1_past"
        )
        return rule_evaluator.evaluate()
    
    
    def evaluate_stop_at_traffic_light(self) -> list[float]:
        """
        Evaluates the rule of stopping at red traffic light for the ego vehicle.
        
        Returns:
            A list of floats representing the evaluated robustness for each time step.
        """    
        rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN2_TOR"
        )
        return rule_evaluator.evaluate()
    
