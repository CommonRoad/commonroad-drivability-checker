from typing import Tuple, Dict, Union, Optional
import os
import logging
import copy

from vehiclemodels import parameters_vehicle2

from crmonitor.common.world import World
from crmonitor.common.helper import load_yaml
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction

from crmonitor.predicates.position import PredSafeDistPrec, PredInFrontOf, PredInSameLane, PredPreceding
from crmonitor.predicates.general import PredCutIn
from crmonitor.predicates.acceleration import PredAbruptBreaking, PredRelAbruptBreaking


class TrafficRuleChecker:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory,
                 planning_problem: PlanningProblem, config_path : str = None) -> None :
        
        """
        a class used to evaluate traffic rule compliance of the ego vehicle at every
        time step.
        Uses the predicates and functions defined in common-road-stl and formalizations on 
        the Interstate Paper of Sebastian Maihofer.
        
        param scenario : the evaluated scenario
        param ego_trajectory : the submitted trajectory for the ego vehicle
        param planning_problem : the mission of the ego vehicle
        config_path : path to the yaml file containing the parameters of the ego vehicle and the other vehicle
        
        """        
        self._scenario = scenario
        self._ego_obstacle_id = scenario.generate_object_id() # To ensure a unique ID for the ego_obstacle/ego_vehicle
        self._ego_trajectory = ego_trajectory
        self._initial_state = planning_problem.initial_state
        # Create ego_obstacle
        self._ego_obstacle = self._create_ego_obstacle(self._ego_trajectory,
                                                        self._initial_state, self._ego_obstacle_id)
        # Add ego obstacle to the scenario and assign lanelets to all obstacles in scenario
        self._scenario.add_objects(self._ego_obstacle)
        self._scenario.assign_obstacles_to_lanelets()
        # Initialize World from scenario
        self._world = World.create_from_scenario(self._scenario)
        # Load configuration for traffic rules
        self._config = self._load_configuration(config_path)
        # Store vehicle IDs
        self._ego_vehicle_id = self._ego_obstacle.obstacle_id  # Use obstacle ID as vehicle ID for the ego vehicle
        self._ego_vehicle = self._world.vehicle_by_id(self._ego_vehicle_id)
        self._other_vehicles = self._world.vehicles # Ego Vehicle included
        self._other_vehicle_ids = [vehicle.id for vehicle in self._other_vehicles]
        # Initialize logger
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("T_R_Checker")
        
        # # # FOR DEBUGGING PURPOSES TO ENSURE THE EGO VEHICLE IS ADDED TO THE WORLD
        # self.logger.info(f"=======Scenario Vehicles========")
        # self.logger.info(f"Ego vehicle ID : {self._ego_vehicle_id}")
        # self.logger.info(f"Other Vehicle IDs")
        # for id in self._other_vehicle_ids:
        #     self.logger.info(id)
          
    def _load_configuration(self, config_path : str) -> Union[Dict, None] :
        
        """ a private method to load the configuration file (a yaml file)
            this function calls the load_yaml
        Args:
            config_path (str): the path to the file in the project
        Returns:
            Union[Dict, None]: a Dict mapping the vehicle paramters to numbers, can be customized
        """    
        # Use default configuration if none is given   
        if config_path is None :
            config_path = os.path.join(os.getcwd(), "commonroad_dc/costs/config.yaml")
            try:
                return load_yaml(config_path)
            except Exception as ex :
                self.logger.error(f"Failed to load configuaration file : {ex}")
                return None
    
      
    def _create_ego_obstacle(self, trajectory: Trajectory,
                             ego_initial_state: PlanningProblem.initial_state,
                             ego_id) -> DynamicObstacle:
        
        """ a private method to create an ego obstacle that spans the given trajectory
            this dynamic obstacle is then the ego vehicle whose behaviour is checked
            NB : this obstacle takes the shape of the Vehicle 2 in CommonRoad assuming KS2 benchmark
        Returns:
            dynamic obstacle object
        """        
        vehicle_2 = parameters_vehicle2.parameters_vehicle2()
        vehicle_shape = Rectangle(width=vehicle_2.w, length=vehicle_2.l)
        
        if trajectory.initial_time_step == ego_initial_state.time_step:
            trajectory_tmp = Trajectory(initial_time_step=trajectory.initial_time_step + 1,
                                    state_list=trajectory.state_list[1:])
        else:
            trajectory_tmp = Trajectory(initial_time_step=trajectory.initial_time_step,
                                        state_list=trajectory.state_list[:])
        assert trajectory_tmp.initial_time_step-1 == ego_initial_state.time_step
        ego_prediction = TrajectoryPrediction(trajectory_tmp, vehicle_shape)
        ego_obstacle = DynamicObstacle(ego_id, ObstacleType.CAR, vehicle_shape,
                                    ego_initial_state, ego_prediction)
        return ego_obstacle 
          
          
    def _has_valid_trajectory(self, vehicle, time_step: int) -> bool:
        """Check if the vehicle has a valid trajectory at the given time step."""
        return time_step in vehicle.states_cr


    def _has_valid_lanelet_assignment(self, vehicle, time_step: int) -> bool:
        """Check if the vehicle has a valid lanelet assignment at the given time step."""
        return vehicle.lanelet_assignment and time_step in vehicle.lanelet_assignment

    # Rule checking methods at all time steps, logger can be uncommented for debugging purposes
  
    def check_safe_distance(self, time_step: int) -> Tuple[bool, Optional[int], float, float]:
        """checks if the ego vehicle complies with the safe distance rule at the current time_step

        Args:
            time_step (int): 

        Returns:
            Tuple[bool, int, float, float]: if safa distance is maintained, preceding vehicle ID, actual distance to preceding vehicle, safe distance threshold
        """        
        
        in_same_lane_pred = PredInSameLane(self._config)
        in_front_of_pred = PredInFrontOf(self._config)
        safe_distance_pred = PredSafeDistPrec(self._config)
        cut_in_pred = PredCutIn(self._config)
        
        distance_to_preceding_vehicle = float('inf')  
        vehicles_in_front_of_ego_ids = []  # List saving the id(s) of the vehicle(s) in front of the ego at the current time_step
        t_c = 1  
        previous_time = max(0, time_step - t_c)   
        # Loop through all other vehicles to find those in the same lane and in front of the ego vehicle
        for other_vehicle_id in self._other_vehicle_ids:
            try:
                if other_vehicle_id == self._ego_vehicle_id:
                    continue  # skip the ego vehicle itself
                other_vehicle = self._world.vehicle_by_id(other_vehicle_id)    
                if not self._has_valid_trajectory(other_vehicle, time_step):
                    continue
                if not self._has_valid_lanelet_assignment(other_vehicle, time_step):
                    continue 
                ego_vehicle_lane = self._ego_vehicle.get_lane(time_step)
                other_vehicle_rear_s = other_vehicle.rear_s(time_step, ego_vehicle_lane)
                if other_vehicle_rear_s is None: # No Information on the rear position of this vehicle at this time step
                    continue
                if time_step not in other_vehicle.lanelet_assignment:
                    continue    
                # Check if the vehicle is in the same lane
                in_same_lane_as_ego = in_same_lane_pred.evaluate_boolean(self._world, time_step,
                                                                         [other_vehicle_id, self._ego_vehicle_id])
                if not in_same_lane_as_ego:
                    continue
                # Check if the vehicle is in front of the ego vehicle
                in_front_of_ego = in_front_of_pred.evaluate_boolean(self._world, time_step, 
                                                                    [self._ego_vehicle_id, other_vehicle_id])
                if not in_front_of_ego:
                    continue 
                vehicles_in_front_of_ego_ids.append(other_vehicle_id)   
            except Exception as ex:
                self.logger.warning(f"Error processing vehicle {other_vehicle_id} for SD Check @ time {time_step}: {ex}")
        # If there are no preceding vehicles, return True ,safe distance trivially maintained
        if len(vehicles_in_front_of_ego_ids) == 0:
           # self.logger.info(f"No obstacle in front of the ego vehicle within sensor range at time {time_step}")
            return True, None, float('inf'), float('inf')
        # Evaluate the minimum distance to the preceding vehicles to get the distance of the vehicle in front
        for vehicle_id in vehicles_in_front_of_ego_ids:
            distance = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                            [self._ego_vehicle_id, vehicle_id])   
            if distance < distance_to_preceding_vehicle:
                distance_to_preceding_vehicle = distance
                preceding_vehicle_id = vehicle_id
        preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id)
        cut_in_happened_p = False # cut_in happend in the previous state
        cut_in_happened_o = False # cut_in happened in some previoud state
        if previous_time in self._ego_vehicle.states_cr and previous_time in preceding_vehicle.states_cr:
            cut_in_happened_p = cut_in_pred.evaluate_boolean(self._world, previous_time, 
                                                            [preceding_vehicle_id, self._ego_vehicle_id])
            valid_time_steps = range(0, previous_time)
            cut_in_happened_o = any([cut_in_pred.evaluate_boolean(self._world, t, 
                                        [preceding_vehicle_id, self._ego_vehicle_id])
                for t in valid_time_steps
            ])
        safe_distance_kept = safe_distance_pred.evaluate_boolean(self._world, time_step, 
                                                                [self._ego_vehicle_id, preceding_vehicle_id])
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept:
          #  self.logger.info(f"Safe Distance Maintained following {other_vehicle_id}, maybe violated only due to a cut-in at previous time: {time_step}")
            return True, preceding_vehicle_id,float('inf'), float('inf')
        distance_to_preceding_vehicle = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                                            [self._ego_vehicle_id, preceding_vehicle_id])
        

        a_min_ego = self._ego_vehicle.vehicle_param.get("a_min")
        a_min_preceding = preceding_vehicle.vehicle_param.get("a_min")
        t_react_ego = self._ego_vehicle.vehicle_param.get("t_react")
        safe_distance_threshold = PredSafeDistPrec.calculate_safe_distance(
            self._ego_vehicle.states_cr[time_step].velocity,
            preceding_vehicle.states_cr[time_step].velocity,
            a_min_preceding,
            a_min_ego,
            t_react_ego
        )
      #  self.logger.info(f"Safe Distance violated while following {preceding_vehicle_id} without cut-in at time: {time_step}")
        return False, preceding_vehicle_id, distance_to_preceding_vehicle, safe_distance_threshold
 
    
    
    def check_no_unnecessary_braking(self, time_step : int) -> Tuple[bool, Optional[int]] :
        """checks if ego vehicle braked abruptly at the current time step

        Args:
            time_step (int): 

        Returns:
            Tuple[bool, float]: if no abrupt braking, offset between the maximum ego vehicle acceleration and the ego acceleration at this time step
        """        
        rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)
        
        abrupt_braking_pred = PredAbruptBreaking(self._config)
        
        safe_distance_kept, preceding_vehicle_id, _, _ = self.check_safe_distance(time_step)
        
        ego_aceleration = self._ego_vehicle.state_list_cr[time_step].acceleration
        
        violation = ego_aceleration - self._config["ego_vehicle_param"]["a_max"]
        
        if preceding_vehicle_id is None:
            braked_abruptly = abrupt_braking_pred.evaluate_boolean(self._world, time_step, 
                                                            [self._ego_vehicle_id, None])
            
            if braked_abruptly:
                return False, violation
        
        braked_abruptly_rel = rel_abrupt_braking_pred.evaluate_boolean(self._world, time_step,
                                                                    [self._ego_vehicle_id, preceding_vehicle_id])
        
       # print(f"Ego acceleration at time {time_step} is {ego_aceleration}m/s²")
        
        if not safe_distance_kept or braked_abruptly_rel:
                return False, violation
        
        return True, None

