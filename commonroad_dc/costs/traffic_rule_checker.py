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
from crmonitor.predicates.velocity import PredLaneSpeedLimit, PredLaneSpeedLimitStar,PredBrSpeedLimit,PredFovSpeedLimit,PredTypeSpeedLimit


class TrafficRuleChecker:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory,
                 planning_problem: PlanningProblem, config_path : str = None) -> None :
        """
        a class used to evaluate traffic rule compliance of the ego vehicle at every
        time step.
        Uses the predicates and functions defined in common-road-stl and formalizations on 
        the Interstate Paper of Sebastian Maierhofer et al.
        
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
        Returns:
            Tuple:
            -if safe distance is maintained : bool
            -preceding vehicle ID, None if no preceding vehicle at time step (Assume: not within sensor range) : Optional[int]
            -actual distance to preceding vehicle : float
            -safe distance threshold : float
        """        
        in_same_lane_pred = PredInSameLane(self._config)
        in_front_of_pred = PredInFrontOf(self._config)
        safe_distance_pred = PredSafeDistPrec(self._config)
        cut_in_pred = PredCutIn(self._config)
        
        distance_to_preceding_vehicle = float('inf')  
        vehicles_in_front_of_ego_ids = []  # List saving the id(s) of the vehicle(s) in front of the ego at the current time_step
        t_c = 3  # According to Maierhofer et al.
        previous_time = max(0, time_step - t_c)   
        
        for other_vehicle_id in self._other_vehicle_ids: # Loop to find the vehicles in same lane and in front of the ego vehicle
            try:
                if other_vehicle_id == self._ego_vehicle_id:
                    continue  # skip the ego vehicle itself
                # skip vehicle(s) with no valid state at the current time_step
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
                # procede with only valid vehicle(s) at this time step   
                # check if the vehicle is in the same lane
                in_same_lane_as_ego = in_same_lane_pred.evaluate_boolean(self._world, time_step,
                                                                         [other_vehicle_id, self._ego_vehicle_id])
                if not in_same_lane_as_ego:
                    continue
                # check if the vehicle is in front of the ego vehicle
                in_front_of_ego = in_front_of_pred.evaluate_boolean(self._world, time_step, 
                                                                    [self._ego_vehicle_id, other_vehicle_id])
                if not in_front_of_ego:
                    continue 
                vehicles_in_front_of_ego_ids.append(other_vehicle_id)   
            except Exception as ex:
                self.logger.warning(f"Error processing vehicle {other_vehicle_id} for SD Check @ time {time_step}: {ex}")
                
        if len(vehicles_in_front_of_ego_ids) == 0: # No vehicle(s) in front => safe distance is trivially maintained
           # self.logger.info(f"No obstacle in front of the ego vehicle within sensor range at time {time_step}")
            return True, None, float('inf'), float('inf')
        
        for vehicle_id in vehicles_in_front_of_ego_ids: # Finding the preceding vehicle using min distance evaluation
            distance = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                            [self._ego_vehicle_id, vehicle_id])   
            if distance < distance_to_preceding_vehicle:
                distance_to_preceding_vehicle = distance
                preceding_vehicle_id = vehicle_id
        preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id) # preceding vehicle found
        # check if cut_in happened
        cut_in_happened_p = False # cut_in happend in the previous state
        cut_in_happened_o = False # cut_in happened in some previous state
        
        if previous_time in self._ego_vehicle.states_cr and previous_time in preceding_vehicle.states_cr:
            cut_in_happened_p = cut_in_pred.evaluate_boolean(self._world, previous_time, 
                                                            [preceding_vehicle_id, self._ego_vehicle_id])
            valid_time_steps = range(0, previous_time)
            cut_in_happened_o = any([cut_in_pred.evaluate_boolean(self._world, t, 
                                        [preceding_vehicle_id, self._ego_vehicle_id])
                for t in valid_time_steps])
        # evaluate if safe distance is kept  
        safe_distance_kept = safe_distance_pred.evaluate_boolean(self._world, time_step, 
                                                                [self._ego_vehicle_id, preceding_vehicle_id])
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept:
          #  self.logger.info(f"Safe Distance Maintained following {other_vehicle_id}, maybe violated only due to a cut-in at previous time: {time_step}")
            return True, preceding_vehicle_id,float('inf'), float('inf')
        distance_to_preceding_vehicle = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                                            [self._ego_vehicle_id, preceding_vehicle_id])
        # evaluate the safe distance value
        a_min_ego = self._ego_vehicle.vehicle_param.get("a_min")
        a_min_preceding = preceding_vehicle.vehicle_param.get("a_min")
        t_react_ego = self._ego_vehicle.vehicle_param.get("t_react")
        safe_distance_threshold = PredSafeDistPrec.calculate_safe_distance(
                                                                self._ego_vehicle.states_cr[time_step].velocity,
                                                                preceding_vehicle.states_cr[time_step].velocity,
                                                                a_min_preceding,
                                                                a_min_ego,
                                                                t_react_ego)
      #  self.logger.info(f"Safe Distance violated while following {preceding_vehicle_id} without cut-in at time: {time_step}")
        return False, preceding_vehicle_id, distance_to_preceding_vehicle, safe_distance_threshold
 
    
    def check_no_unnecessary_braking(self, time_step: int) -> Tuple[bool, Optional[float]]:
        """cheks if the ego vehicle braked abruptly and unnecessarily at the given time step.
        Returns:
            Tuple:
            - if no unnecessary braking (True if no unnecessary braking) : bool
            - penalty or violation measure for unnecessary braking (None if no violation) : Optional[float]
        """
        abrupt_braking_pred = PredAbruptBreaking(self._config)
        rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)
        safe_distance_kept, preceding_vehicle_id, _, _ = self.check_safe_distance(time_step)
        ego_acceleration = self._ego_vehicle.states_cr[time_step].acceleration # ego acceleration at the  current time_step
        a_abrupt_threshold = self._config["ego_vehicle_param"]["a_abrupt"]
        
        # Compare acceleration to the defined threshold and set violation
        if ego_acceleration < -a_abrupt_threshold:
            violation = abs(ego_acceleration + a_abrupt_threshold)
        else:
            violation = 0  # if acceleration is within limits => No violation
       # if no preceding vehicle the ego vehicle should not brake abruptly
        if preceding_vehicle_id is None:
            braked_abruptly = abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            if braked_abruptly:
                self.logger.info(f"Ego braked abruptly without a preceding vehicle at time {time_step}.")
                return False, violation  # Penalize unnecessary braking
       # Preceding vehicle exists, check abrupt braking relative to the preceding vehicle
        elif preceding_vehicle_id is not None:
            try:
                braked_abruptly_rel = rel_abrupt_braking_pred.evaluate_boolean(self._world, time_step,
                                                                            [self._ego_vehicle_id, preceding_vehicle_id])
            except AttributeError as e:
                self.logger.warning(f"Error: Preceding vehicle not found or invalid at time {time_step}: {e}")
                return True, None  # unnecessary braking not checked since there's no valid preceding vehicle
                   
            if not safe_distance_kept or braked_abruptly_rel:
               # self.logger.info(f"Ego vehicle braked abruptly relative to preceding vehicle at time {time_step}.")
                return False, violation  # Penalize for braking abruptly
        # No abrupt or unnecessary braking detected
       # self.logger.info(f"Ego acceleration at time {time_step} is {ego_acceleration} m/s². No unnecessary braking detected.")
        return True, None  # No penalty if braking is justified


    def check_maximum_speed_limit(self, time_step: int) -> Tuple[bool, Optional[float]]:
        """checks if maximum speed limit of the lane is violated
        Returns:
            Tuple:
            -if speed limit is exceeded (True if no violation) : bool
            -violation : float
        """
        lane_speed_limit_pred = PredLaneSpeedLimit(self._config)
        brake_speed_limit_pred = PredBrSpeedLimit(self._config)    # method 'get_speed_limit()' loads value from the config file according to the implementation in stl-monitor
        fov_speed_limit_pred = PredFovSpeedLimit(self._config)     # method 'get_speed_limit()' loads value from the given config file 
        type_speed_limit_pred = PredTypeSpeedLimit(self._config)   # method 'get_speed_limit()' loads value from the given config file  
        
        lane_speed_limit = lane_speed_limit_pred.get_speed_limit(self._world, time_step,
                                                                 [self._ego_vehicle_id, None])   
        #brake_speed_limit = brake_speed_limit_pred.get_speed_limit(self._world, time_step,
        #                                                           [self._ego_vehicle_id, None])
        #fov_speed_limit = fov_speed_limit_pred.get_speed_limit(self._world, time_step,
        #                                                      [self._ego_vehicle_id, None])
           
        # to save computation time i simply load the values from the config file    
        brake_speed_limit = self._config["ego_vehicle_param"]["braking_speed_limit"] # Use config parameter
        fov_speed_limit = self._config["ego_vehicle_param"]["fov_speed_limit"] # Use config file parameter 
        type_speed_limit = self._config["ego_vehicle_param"]["v_max"]  # Assuming ego vehicle uses KS2 model just as in config file
        
        ego_velocity = self._ego_vehicle.states_cr[time_step].velocity  # Velocity of the ego vehicle at this time
        
        # if lane_speed_limit is None assume lane has no speed limit or lane speed limit is 'inf' 
        lane_speed_limit_kept = (ego_velocity <= lane_speed_limit) if lane_speed_limit is not None else True
        lane_speed_limit =  float('inf') if lane_speed_limit is None else lane_speed_limit
        
        # if brake_speed_limit is None, assume there is no brake speed limit or brake speed limit is 'inf'      
        brake_speed_limit_kept = (ego_velocity <= brake_speed_limit) if brake_speed_limit is not None else True
        brake_speed_limit = float('inf') if brake_speed_limit is None else brake_speed_limit
            
        fov_speed_limit_kept = (ego_velocity <= fov_speed_limit)    
        type_speed_limit_kept = (ego_velocity <= type_speed_limit)
        
        # # FOR DEBUGGING PURPOSES
        # self.logger.info(f"EGO VEL at {time_step}: {ego_velocity}")
        # self.logger.info(f"Lane Speed Limit : {lane_speed_limit}")
        # self.logger.info(f"Brake Speed Limit : {brake_speed_limit}")
        # self.logger.info(f"Fov Speed Limit : {fov_speed_limit}")
        # self.logger.info(f"Type Speed Limit : {type_speed_limit}")
        
        if lane_speed_limit_kept and brake_speed_limit_kept and fov_speed_limit_kept and type_speed_limit_kept:  # Temporal Logic from the Paper of Sebastian Maierhofer et al.
            return True, None
        else:
            violation = max((ego_velocity-brake_speed_limit),
                            (ego_velocity-lane_speed_limit),
                            (ego_velocity-fov_speed_limit),
                            (ego_velocity-type_speed_limit))
            return False, violation