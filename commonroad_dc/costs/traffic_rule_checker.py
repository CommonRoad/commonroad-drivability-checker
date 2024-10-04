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
from crmonitor.predicates.general import PredCutIn, PredInCongestion
from crmonitor.predicates.acceleration import PredAbruptBreaking, PredRelAbruptBreaking
from crmonitor.predicates.velocity import (PredLaneSpeedLimit, PredLaneSpeedLimitStar,PredBrSpeedLimit,
                                           PredFovSpeedLimit,PredTypeSpeedLimit, PredSlowLeadingVehicle,
                                           PredPreservesTrafficFlow, PredInStandStill, PredExistStandingLeadingVehicle)


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
        self._ego_obstacle = self._create_ego_obstacle(self._ego_trajectory,
                                                        self._initial_state, self._ego_obstacle_id) # Create ego_obstacle
        self._scenario.add_objects(self._ego_obstacle)              # Add ego obstacle to the scenario
        self._scenario.assign_obstacles_to_lanelets()               # Assign lanelets to all obstacles in scenario
        self._world = World.create_from_scenario(self._scenario)    # Initialize World from scenario
        self._config = self._load_configuration(config_path)        # Load configuration for traffic rules
        # Store vehicle IDs
        self._ego_vehicle_id = self._ego_obstacle.obstacle_id       # Use obstacle ID as vehicle ID for the ego vehicle
        self._ego_vehicle = self._world.vehicle_by_id(self._ego_vehicle_id)
        self._other_vehicles = self._world.vehicles                 # Ego Vehicle included    
        logging.basicConfig(level=logging.INFO)                     # Initialize logger
        self.logger = logging.getLogger("T_R_Checker")
        
        # # # FOR DEBUGGING PURPOSES TO ENSURE THE EGO VEHICLE IS ADDED TO THE WORLD
        # self.logger.info(f"=======Scenario Vehicles========")
        # self.logger.info(f"Ego vehicle ID : {self._ego_vehicle_id}")
        # self.logger.info(f"Other Vehicle IDs")
        # for id in self._other_vehicle_ids:
        #     self.logger.info(id)
    
    
    def _load_configuration(self, config_path : str) -> Union[Dict, None] :
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
     
    #---------------checker_methods--defined--for--each--rule------------
     
    #--------------R_G_1--------------------
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
        precede_pred = PredPreceding(self._config)
        
        distance_to_preceding_vehicle = float('inf')  
        vehicles_in_front_of_ego_ids = []  # List saving the id(s) of the vehicle(s) in front of the ego at the current time_step
        t_c = 3  # According to Maierhofer et al.
        previous_time = max(0, time_step - t_c)  
        preceding_vehicle_id = None # Initialize preceding vehile ID
        
        for other_vehicle_id in self._world.vehicle_ids_for_time_step(time_step):
            try:
                if other_vehicle_id == self._ego_vehicle_id:
                    continue  # skip the ego vehicle itself 
                other_vehicle = self._world.vehicle_by_id(other_vehicle_id)    
                ego_vehicle_lane = self._ego_vehicle.get_lane(time_step)
                other_vehicle_rear_s = other_vehicle.rear_s(time_step, ego_vehicle_lane)
                if other_vehicle_rear_s is None:
                     continue  # skip vehicl(s) without information on the rear position       
                in_same_lane_as_ego = in_same_lane_pred.evaluate_boolean(self._world, time_step,
                                                                         [other_vehicle_id, self._ego_vehicle_id])   
                if not in_same_lane_as_ego:
                    continue  # skip vehicle(s) not in same lane as ego
                in_front_of_ego = in_front_of_pred.evaluate_boolean(self._world, time_step, 
                                                                    [self._ego_vehicle_id, other_vehicle_id])
                if not in_front_of_ego:
                    continue  # skip vehicle(s) not infront of ego  
                vehicles_in_front_of_ego_ids.append(other_vehicle_id)   # extract vehicles in same lane and in front of ego
            except Exception as ex:
                self.logger.warning(f"Error processing vehicle {other_vehicle_id} for SD Check @ time {time_step}: {ex}")     
        if len(vehicles_in_front_of_ego_ids) == 0: # No vehicle(s) in front => safe distance is trivially maintained
            return True, None, float('inf'), float('inf')
        elif len(vehicles_in_front_of_ego_ids) == 1:  # Only one vehicle in same lane and infront of ego => this vehicle precedes the ego
            preceding_vehicle_id = vehicles_in_front_of_ego_ids[0]
        else:
            for vehicle_id in vehicles_in_front_of_ego_ids: # Finding the preceding vehicle using min distance evaluation
                distance = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                                [self._ego_vehicle_id, vehicle_id])   
                if distance < distance_to_preceding_vehicle:
                    distance_to_preceding_vehicle = distance
                    preceding_vehicle_id = vehicle_id
        preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id) # preceding vehicle found
        # check if cut_in happened
        cut_in_happened_p = False # cut_in happened in the previous state
        cut_in_happened_o = False # cut_in happened in some previous state 
        if previous_time in preceding_vehicle.states_cr:
            cut_in_happened_p = cut_in_pred.evaluate_boolean(self._world, previous_time, 
                                                            [preceding_vehicle_id, self._ego_vehicle_id])
            valid_time_steps = range(0, previous_time)
            
            cut_in_happened_o = any([cut_in_pred.evaluate_boolean(self._world, t, 
                                        [preceding_vehicle_id, self._ego_vehicle_id])
                for t in valid_time_steps])
        # evaluate if safe distance is kept  
        safe_distance_kept = safe_distance_pred.evaluate_boolean(self._world, time_step, 
                                                                [self._ego_vehicle_id, preceding_vehicle_id])
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept: # Temporal Logic resolution in CNF
          #  self.logger.info(f"Safe Distance Maintained following {other_vehicle_id}, maybe violated only due to a cut-in at previous time: {time_step}")
            return True, preceding_vehicle_id,float('inf'), float('inf')
        else: 
            # safe distance violated, check the actual and safe distance value
            distance_to_preceding_vehicle = in_front_of_pred.evaluate_robustness(self._world, time_step, 
                                                                                [self._ego_vehicle_id, preceding_vehicle_id])
            # evaluate the safe distance value
            a_min_ego = self._ego_vehicle.vehicle_param.get("a_min")
            a_min_preceding = preceding_vehicle.vehicle_param.get("a_min")
            t_react_ego = self._ego_vehicle.vehicle_param.get("t_react")
            safe_distance_threshold = PredSafeDistPrec.calculate_safe_distance(self._ego_vehicle.states_cr[time_step].velocity,
                                                                            preceding_vehicle.states_cr[time_step].velocity,
                                                                            a_min_preceding,
                                                                            a_min_ego,
                                                                            t_react_ego)
        #  self.logger.info(f"Safe Distance violated while following {preceding_vehicle_id} without cut-in at time: {time_step}")
            return False, preceding_vehicle_id, distance_to_preceding_vehicle, safe_distance_threshold
 
 
    #--------------R_G_2--------------------
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
            
        if preceding_vehicle_id is None: #  Case 1 No preceding vehicle, check only abrupt braking
            braked_abruptly = abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            
            if not braked_abruptly:
                return True, None
            else:
                # Using robustness as the cost of violation
                robustness = abrupt_braking_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                return False, abs(robustness)
        
        else: # Case 2 : There is a preceding vehicle, check both abrupt braking and relative abrupt braking
            braked_abruptly = abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            braked_abruptly_rel = rel_abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])
            # Using CNF resolution of the temporal logic 
            condition_1 = (not braked_abruptly or preceding_vehicle_id)
            condition_2 = (not braked_abruptly or not safe_distance_kept or not braked_abruptly_rel)
            
            if condition_1 and condition_2: # True => abrupt relative braking
                robustness = rel_abrupt_braking_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])
                return False, abs(robustness)
            else:
                # No relative abrupt braking
                return True, None


    #--------------R_G_3--------------------
    def check_maximum_speed_limit(self, time_step: int) -> Tuple[bool, Optional[float]]:
        """checks if maximum speed limit of the lane is violated
        Returns:
            Tuple:
            - if speed limit is exceeded (True if no violation): bool
            - violation: float
        """
        lane_speed_limit_pred = PredLaneSpeedLimit(self._config)
        brake_speed_limit_pred = PredBrSpeedLimit(self._config)    
        fov_speed_limit_pred = PredFovSpeedLimit(self._config)    
        type_speed_limit_pred = PredTypeSpeedLimit(self._config)

        # Get speed limits from predicates
        lane_speed_limit = lane_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        brake_speed_limit = brake_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        fov_speed_limit = fov_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        type_speed_limit = type_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])

        # Assume no speed limits if they are None
        lane_speed_limit = float('inf') if lane_speed_limit is None else lane_speed_limit
        brake_speed_limit = float('inf') if brake_speed_limit is None else brake_speed_limit
        fov_speed_limit = float('inf') if fov_speed_limit is None else fov_speed_limit
        type_speed_limit = float('inf') if type_speed_limit is None else type_speed_limit

        # Ego vehicle's velocity
        ego_velocity = self._ego_vehicle.states_cr[time_step].velocity

        # Check if each speed limit is respected
        lane_speed_limit_kept = (ego_velocity <= lane_speed_limit)
        brake_speed_limit_kept = (ego_velocity <= brake_speed_limit)
        fov_speed_limit_kept = (ego_velocity <= fov_speed_limit)
        type_speed_limit_kept = (ego_velocity <= type_speed_limit)

        # # for debugging
        # self.logger.info(f"EGO VEL at {time_step}: {ego_velocity}")
        # self.logger.info(f"Lane Speed Limit: {lane_speed_limit}")
        # self.logger.info(f"Brake Speed Limit: {brake_speed_limit}")
        # self.logger.info(f"FOV Speed Limit: {fov_speed_limit}")
        # self.logger.info(f"Type Speed Limit: {type_speed_limit}")

        # Resolution into CNF form 
        if lane_speed_limit_kept and brake_speed_limit_kept and fov_speed_limit_kept and type_speed_limit_kept:
            return True, None
        else:
            # Return maximum violation
            violation = max(ego_velocity - brake_speed_limit,
                            ego_velocity - lane_speed_limit,
                            ego_velocity - fov_speed_limit,
                            ego_velocity - type_speed_limit)
            return False, violation
  
  
    #--------------R_G_4--------------------    
    def check_traffic_flow(self, time_step: int) -> Tuple[bool, Optional[float]]:
        """checks if ego vehicle maintains traffic flow
        Returns:
              Tuple:
              - if traffic flow is mainted (True if not violated): bool
              - violation : float
        """         
        slow_leading_vehicle_pred = PredSlowLeadingVehicle(self._config)
        preserves_traffic_flow_pred = PredPreservesTrafficFlow(self._config)
        _, preceding_vehicle_id, _, _ = self.check_safe_distance(time_step)
        
        if preceding_vehicle_id is None: # No leading vehicle => No slow leading vehicle, check only traffic flow preservation
            ego_preserves_traffic_flow = preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step, 
                                                                                  [self._ego_vehicle_id, None])
            if ego_preserves_traffic_flow:
                return True, None
            else:
                robustness = preserves_traffic_flow_pred.evaluate_robustness(self._world, time_step, 
                                                                             [self._ego_vehicle_id, None])
                return False, abs(robustness)
            
        else:
            # There is a leading vehicle, check if there is a slow leading vehicle
            try: 
                there_is_a_slow_leading_vehicle = slow_leading_vehicle_pred.evaluate_boolean(self._world, time_step,
                                                                                        [self._ego_vehicle_id, None])  # Line doesnt work for some scenarios 
                 
                ego_preserves_traffic_flow = preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step, 
                                                                                     [self._ego_vehicle_id, None]) 
        
                if there_is_a_slow_leading_vehicle or ego_preserves_traffic_flow: # Resolution of temporal logic in CNF
                    return True, None 
                else:
                    # There is a violation 
                    robustness = slow_leading_vehicle_pred.evaluate_robustness(self._world, time_step, 
                                                                                [self._ego_vehicle_id, None])
                    print(f"Robustness : {robustness}")
                    return False, abs(robustness)     
            except TypeError as e:
                self.logger.info(f"Error processing vehicle for traffic_flow check @ time {time_step} returning True--- : {e}")
                return True, None
      
            
    #--------------R_I_1--------------------    
    def check_no_stopping(self, time_step: int) -> Tuple[bool, Optional[float]]:
        """checks if ego vehicle stopped unnecessarily
        Returns:
              Tuple:
              - if no stopping is mainted (True if not violated): bool
              - violation : float
        """         
        in_stand_still_pred = PredInStandStill(self._config)
        standing_leading_vehicle_pred = PredExistStandingLeadingVehicle(self._config)
        in_congestion_pred = PredInCongestion(self._config)

        _, preceding_vehicle_id, _, _ = self.check_safe_distance(time_step)

        if preceding_vehicle_id is None:  # No preceding vehicle => check if the ego vehicle is standing still
            ego_in_stand_still = in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

            if ego_in_stand_still:
                robustness = in_stand_still_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                return False, robustness  # Unnecessary stopping
            else:
                return True, None  # No stopping detected

        else:  # There is a leading vehicle, perform extensive check
            try:
                ego_in_congestion = in_congestion_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
                there_is_a_standing_leading_vehicle = standing_leading_vehicle_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
                ego_in_stand_still = in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

                # Temporal Logic Resolution into CNF
                condition_1 = (not ego_in_stand_still or ego_in_congestion)  # If ego is in standstill, ensure it's in congestion
                condition_2 = (not ego_in_stand_still or not there_is_a_standing_leading_vehicle)  # If in standstill, check if leading vehicle is also standing still

                if not (condition_1 and condition_2):  # If either condition fails, there is unnecessary stopping
                    robustness = in_stand_still_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                    return False, robustness
                else:
                    return True, None  # No unnecessary stopping detected
            except Exception as ex:
                self.logger.info(f"Error processing vehicle for no_stopping check @ time {time_step}: returning True {ex}")
                return True, None # Assuming no violation
         
         
    #--------------R_I_2--------------------    
    def check_driving_faster_than_left_traffic(self, time_step: int) -> Tuple[bool, Optional[float]]:
        pass

    
    #--------------R_I_3--------------------    
    def check_reversing_and_turning(self, time_step: int) -> Tuple[bool, Optional[float]]:
        pass
    
    
    #--------------R_I_4--------------------    
    def check_emergency_lane(self, time_step: int) -> Tuple[bool, Optional[float]]:
        pass
    
    
    #--------------R_I_5--------------------    
    def check_consideration_of_entering_vehicles(self, time_step: int) -> Tuple[bool, Optional[float]]:
        pass