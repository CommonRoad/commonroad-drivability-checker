from typing import Tuple, Optional
import os
import logging
import copy
from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap

from vehiclemodels import parameters_vehicle2

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction

from crmonitor.common.world import World
from crmonitor.predicates.general import (PredCutIn, PredInCongestion, PredInSlowMovingTraffic, PredInQueueOfVehicles,
                                          PredMakesUTurn, PredInterstateBroadEnough, PredSingleLane, PredInIntersection,
                                          PredTrafficLightRed, PredStopLineInFront, PredOnRightTurn) 
                                           
from crmonitor.predicates.position import (PredSafeDistPrec, PredInFrontOf, PredInSameLane, PredPreceding, PredLeftOf,
                                           PredRightOfBroadLaneMarking, PredLeftOfBroadLaneMarking, PredOnAccessRamp,
                                           PredOnMainCarriageway, PredOnShoulder, PredInRightmostLane, PredInLeftmostLane,
                                           PredDrivesLeftmost, PredDrivesRightmost, PredMainCarriageWayRightLane)
                                           
from crmonitor.predicates.velocity import (PredLaneSpeedLimit,PredBrSpeedLimit,PredFovSpeedLimit,PredTypeSpeedLimit, 
                                           PredSlowLeadingVehicle,PredPreservesTrafficFlow, PredInStandStill, PredExistStandingLeadingVehicle,
                                           PredDrivesFaster, PredDrivesWithSlightlyHigherSpeed, PredReverses)

from crmonitor.predicates.acceleration import PredAbruptBreaking, PredRelAbruptBreaking


class TrafficRuleChecker:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory,
                 planning_problem: PlanningProblem, config_path: str = None) -> None:
        """
        Initializes an instance of the TrafficRuleChecker class.

        Args:
            scenario (Scenario): The evaluated scenario.
            ego_trajectory (Trajectory): The submitted trajectory for the ego vehicle.
            planning_problem (PlanningProblem): The mission of the ego vehicle.
            config_path (str, optional): Path to the YAML file containing the parameters of the ego vehicle and other vehicles.

        Attributes:
            _scenario (Scenario): The evaluated scenario.
            _ego_obstacle_id (int): The unique ID for the ego obstacle/ego vehicle.
            _ego_trajectory (Trajectory): The submitted trajectory for the ego vehicle.
            _initial_state (State): The initial state of the planning problem.
            _ego_obstacle (Obstacle): The ego obstacle created from the ego trajectory and initial state.
            _world (World): The world created from the scenario.
            _config (dict): The configuration for traffic rules and vehicle model.
            _ego_vehicle_id (int): The vehicle ID for the ego vehicle.
            _ego_vehicle (Vehicle): The ego vehicle in the world.
            _other_vehicles (List[Vehicle]): The list of other vehicles in the world.
            logger (Logger): The logger for the TrafficRuleChecker class.
        """
        self._scenario = scenario
        self._ego_obstacle_id = scenario.generate_object_id()
        self._ego_trajectory = ego_trajectory
        self._initial_state = planning_problem.initial_state
        self._ego_obstacle = self._create_ego_obstacle(self._ego_trajectory, self._initial_state, self._ego_obstacle_id)                                               
        self._scenario.add_objects(self._ego_obstacle)
        self._scenario.assign_obstacles_to_lanelets()
        self._world = copy.deepcopy(World.create_from_scenario(self._scenario))
        self._config = self._load_configuration(config_path)
        self._ego_vehicle_id = self._ego_obstacle.obstacle_id
        self._ego_vehicle = self._world.vehicle_by_id(self._ego_vehicle_id)
        self._logger = logging.getLogger("T_R_Checker")
        
        # Update vehicle parameters
        self._update_vehicle_parameters()
        
    #--------------------------private_methods--------------------------
    
    def _update_vehicle_parameters(self) -> None:
        """
        Updates the vehicle parameters for the ego vehicle and other vehicles in the world.
        """
        # Update other vehicles' parameters
        self._other_vehicles = self._world.vehicles
        for vehicle in self._other_vehicles:
            if vehicle.id == self._ego_vehicle_id:
                continue  # Skip the ego vehicle
            vehicle.vehicle_param.update(self._config["other_vehicles_param"])
        
        # Update ego vehicle's parameters
        self._ego_vehicle.vehicle_param.update(self._config["ego_vehicle_param"])

    def _load_configuration(self, config_path: str) -> CommentedMap:
        yaml_loader = YAML()
        if config_path is None:
            config_path = os.path.join(os.getcwd(), "commonroad_dc/costs/config.yaml")
        try:
            with open(config_path, 'r') as file:
                config = yaml_loader.load(file)
                if not isinstance(config, CommentedMap):
                    raise TypeError("The configuration should be of type CommentedMap.")
                return config
        except Exception as ex:
            raise RuntimeError(f"Configuration loading error: {ex}")
        
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
     
    #-----------------------GENERAL RULES--------------------  
   
    #--------------R_G_1--------------------
    def check_safe_distance(self, time_step: int) -> Tuple[bool, Optional[int], float]:
        """
        Checks if the safe distance is maintained between the ego vehicle and the preceding vehicle at the given time step.

        Returns:
            Tuple[bool, Optional[int], float]: A tuple containing the following:
                - A boolean indicating whether the safe distance is maintained.
                - The ID of the preceding vehicle (if any).
                - The absolute value of the robustness of the safe distance (negative value indicates non-compliance).
        """
        # Initialize predicates 
        in_same_lane_pred = PredInSameLane(self._config)
        in_front_of_pred = PredInFrontOf(self._config)
        safe_distance_pred = PredSafeDistPrec(self._config)
        cut_in_pred = PredCutIn(self._config)
        precede_pred = PredPreceding(self._config) # Not used in this method
          
        vehicles_in_front_of_ego = [
                                    vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
                                    if vehicle_id != self._ego_vehicle_id and in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
                                    vehicle_id for vehicle_id in vehicles_in_front_of_ego 
                                    if in_same_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
                    
        preceding_vehicle_id = None # Initialize preceding vehile ID
        if len(vehicles_in_front_of_and_in_same_lane_as_ego) == 0: # No vehicle(s) in front => safe distance is trivially maintained
            return True, None, 0.0 # safe distance is maintained given no preceding vehicle
        
        elif len(vehicles_in_front_of_and_in_same_lane_as_ego) > 0:
            sorted_vehicles = sorted(
                vehicles_in_front_of_and_in_same_lane_as_ego,
                key=lambda vehicle_id: in_front_of_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
            )
            preceding_vehicle_id = sorted_vehicles[0]  # Select the closest vehicle interms of distance
        
        # Perform cut-in check
        t_c = 3  # According to Maierhofer et al.
        previous_start_time = max(0, time_step - t_c)
        valid_time_steps = range(previous_start_time, time_step) 
        cut_in_happened_p = cut_in_pred.evaluate_boolean(self._world, max(0, time_step - 1), [preceding_vehicle_id, self._ego_vehicle_id]) # Check if cut-in happened at previous time step
        cut_in_happened_o = any([cut_in_pred.evaluate_boolean(self._world, t, [preceding_vehicle_id, self._ego_vehicle_id]) for t in valid_time_steps]) # Check if cut-in happened at some time step in the past
                                           
        # evaluate if safe distance is kept  
        safe_distance_kept = safe_distance_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])  
                                                              
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept: # Temporal Logic resolution in CNF
          #  self.logger.info(f"Safe Distance Maintained following {other_vehicle_id}, maybe violated only due to a cut-in at previous time: {time_step}")
            return True, preceding_vehicle_id, 0.0  # safe distance is maintained given a preceding vehicle
        
        else: # Safe distance is violated, when cut_in_happened_o and cut_in_happened_p and safe_distance_kept are False
            safe_distance_robusness = safe_distance_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]) # returns (actual_distance - safe_distance) => -ve value for non-compliance
            return False, preceding_vehicle_id, abs(safe_distance_robusness)
 
 
    #--------------R_G_2--------------------
    def check_no_unnecessary_braking(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if there is any unnecessary braking based on the given time step.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean value indicating if there is any unnecessary braking,
            and a float value representing the cost of violation. If there is no violation, the second
            element of the tuple is 0.0.
        """        
       # Initialize predicates
        abrupt_braking_pred = PredAbruptBreaking(self._config)
        rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)
        safe_distance_kept, preceding_vehicle_id, _ = self.check_safe_distance(time_step)
            
        if preceding_vehicle_id is None: #  Case 1 No preceding vehicle, check only abrupt braking
            braked_abruptly = abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            
            if not braked_abruptly:
                return True, 0.0
            else:
                # Using robustness as the cost of violation, TODO cost to be modelled as a function of the abruptness of braking
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
    def check_maximum_speed_limit(self, time_step: int) -> Tuple[bool, float]:
        """
        Check if the ego vehicle's speed is within the maximum speed limits.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean indicating if the speed limits are respected
            and the maximum violation if the speed limits are not respected.
        """
        # Initialize predicates
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

        if lane_speed_limit_kept and brake_speed_limit_kept and fov_speed_limit_kept and type_speed_limit_kept: # Resolution into CNF 
            return True, 0.0
        else:
            # Return maximum violation,TODO can be modelled later as a function of the speed limits
            violation = max(ego_velocity - brake_speed_limit,
                            ego_velocity - lane_speed_limit,
                            ego_velocity - fov_speed_limit,
                            ego_velocity - type_speed_limit)
            return False, violation
  
  
    #--------------R_G_4--------------------    
    def check_traffic_flow(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks the traffic flow at the given time step.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean value indicating whether the traffic flow is
            preserved or not, and a float value representing the robustness of the traffic flow preservation.
            If the traffic flow is preserved, the second element of the tuple is 0.0.
        """
        # Initialize predicates   
        slow_leading_vehicle_pred = PredSlowLeadingVehicle(self._config)
        preserves_traffic_flow_pred = PredPreservesTrafficFlow(self._config)
        _, preceding_vehicle_id, _ = self.check_safe_distance(time_step)
        
        if preceding_vehicle_id is None: # No leading vehicle => No slow leading vehicle, check only traffic flow preservation
            ego_preserves_traffic_flow = preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step, 
                                                                                  [self._ego_vehicle_id, None])
            if ego_preserves_traffic_flow:
                return True, 0.0
            else: # TODO cost to be modelled as a function of the robustness of the traffic flow preservation
                robustness = preserves_traffic_flow_pred.evaluate_robustness(self._world, time_step, 
                                                                             [self._ego_vehicle_id, None])
                return False, abs(robustness)
              
        else:
            # There is a leading vehicle, check if there is a slow leading vehicle
            try: 
                there_is_a_slow_leading_vehicle = slow_leading_vehicle_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, None])  # Line doesnt work for some scenarios 
                                                                                        
                ego_preserves_traffic_flow = preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, None]) 
                                                                                     
                if there_is_a_slow_leading_vehicle or ego_preserves_traffic_flow: # Resolution into CNF
                    return True, 0.0 
                else:
                    # There is a violation 
                    robustness = preserves_traffic_flow_pred.evaluate_robustness(self._world, time_step,[self._ego_vehicle_id, None])
                         
                    return False, abs(robustness)     
                
            except TypeError as e:
                self._logger.info(f"Error processing vehicle for traffic_flow check @ time {time_step} returning True--- : {e}")
        
                return True, 0.0
            
      
    #-----------------------INTERSTATE RULES--------------------
            
    #--------------R_I_1--------------------    
    def check_no_stopping(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if the ego vehicle is unnecessarily stopping at the given time step.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean value indicating whether there is unnecessary stopping,
            and a float value representing the robustness of the stopping behavior. If there is no unnecessary stopping,
            the boolean value will be True and the float value will be 0.0.
        """
        # Initialize predicates
        in_stand_still_pred = PredInStandStill(self._config)
        standing_leading_vehicle_pred = PredExistStandingLeadingVehicle(self._config)
        in_congestion_pred = PredInCongestion(self._config)

        _, preceding_vehicle_id, _ = self.check_safe_distance(time_step)

        if preceding_vehicle_id is None:  # No preceding vehicle => check if the ego vehicle is standing still
            ego_in_stand_still = in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

            if ego_in_stand_still: # TODO cost to be modelled as a function of the robustness of the stopping behavior
                robustness = in_stand_still_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                return False, robustness  # Unnecessary stopping
            else:
                return True, 0.0  # No stopping detected

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
                    return True, 0.0  # No unnecessary stopping detected
            except Exception as ex:
                self._logger.info(f"Error processing vehicle for no_stopping check @ time {time_step}: returning True {ex}")
                return True, 0.0 # Assuming no violation
         
         
    #--------------R_I_2--------------------    
    def check_driving_faster_than_left_traffic(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if the vehicle is driving faster than the traffic on its left.

        Returns:
            Tuple[bool, Optional[float]]: A tuple containing:
                - A boolean indicating whether the vehicle is driving faster than the traffic on its left.
                - An optional float value representing the speed difference if the vehicle is driving faster.
        """
        # Initialize predicates
        left_of_pred = PredLeftOf(self._config)
        drives_faster_pred = PredDrivesFaster(self._config)
        in_congestion_pred = PredInCongestion(self._config)
        in_slow_moving_traffic_pred = PredInSlowMovingTraffic(self._config)
        in_queue_pred = PredInQueueOfVehicles(self._config)
        drives_slightly_higher_speed_pred = PredDrivesWithSlightlyHigherSpeed(self._config)
        right_of_broad_lane_marking_pred = PredRightOfBroadLaneMarking(self._config)
        left_of_broad_lane_marking_pred = PredLeftOfBroadLaneMarking(self._config)
        on_access_ramp_pred = PredOnAccessRamp(self._config)
        on_main_carriageway_pred = PredOnMainCarriageway(self._config)
        
        # List all vehicles on the left of the ego vehicle at the this time step
        vehicles_on_the_left_of_ego = [
            other_vehicle_id for other_vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if other_vehicle_id != self._ego_vehicle_id and left_of_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id, self._ego_vehicle_id])
        ]

        if not vehicles_on_the_left_of_ego:  # No vehicles on the left means => trivially no violation
            return True, 0.0
        # Check if the ego vehicle is driving faster than any vehicle on its left, returns the first violation
        for other_vehicle_id in vehicles_on_the_left_of_ego:
            ego_drives_faster = drives_faster_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            if not ego_drives_faster:  # Ego is not driving faster than this vehicle, continue checking others
                continue

            # Check if the conditions of safe overtaking are met
            other_in_queue = in_queue_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            other_in_congestion = in_congestion_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            other_in_slow_traffic = in_slow_moving_traffic_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            ego_slightly_faster = drives_slightly_higher_speed_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            ego_right_broad_marking = right_of_broad_lane_marking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            other_left_broad_marking = left_of_broad_lane_marking_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            ego_on_access_ramp = on_access_ramp_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            other_on_main_carriageway = on_main_carriageway_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            
            # According to the paper, the following conditions should be met if the ego vehicle is driving faster than the traffic on its left
            condition_1 = (other_in_queue or other_in_congestion or other_in_slow_traffic) and ego_slightly_faster
            condition_2 = ego_right_broad_marking and other_left_broad_marking
            condition_3 = ego_on_access_ramp and other_on_main_carriageway and not (other_in_queue or other_in_congestion or other_in_slow_traffic)

            # If any of the conditions are met, there is no violation
            if condition_1 or condition_2 or condition_3:
                return True, 0.0  # No violation

            # If none of the conditions are met, calculate the speed difference as a violation TODO cost to be modelled as a function of the speed difference
            speed_difference = drives_faster_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            return False, speed_difference  # Violation occurred

        # If no vehicles cause a violation, return no violation
        return True, 0.0
                
    
    #--------------R_I_3--------------------    
    def check_reversing_and_turning(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if the vehicle is making a U-turn or reversing at the current time step.

        Returns:
            Tuple[bool, Optional[float]]: A tuple containing a boolean value indicating if the vehicle is making a U-turn or reversing,
            and an optional float value representing the unprocessed cost associated with violating the traffic rule.
        """
        # Initialize predicates
        makes_u_turn_pred = PredMakesUTurn(self._config)
        reverses_pred = PredReverses(self._config)
        
        ego_makes_u_turn = makes_u_turn_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_reverses = reverses_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        
        if not ego_makes_u_turn and not ego_reverses:
            return True, 0.0
        else:
            # Return the robustness as the cost of violation # TODO cost to be modelled as a function of the robustness
            robustness = makes_u_turn_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id]) + \
                         reverses_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
            return False, robustness
        
    
    #--------------R_I_4--------------------    
    def check_emergency_lane(self, time_step: int) -> Tuple[bool, float]:
        """
        Check if the vehicle is allowed to use the emergency lane.

        Returns:
            Tuple[bool, Optional[float]]: A tuple containing a boolean value indicating whether the vehicle is allowed to use the emergency lane,
            and an optional float value representing the unprocessed cost associated with violation.
        """
        # Initialize predicates
        in_congestion_pred = PredInCongestion(self._config)
        in_slow_moving_traffic_pred = PredInSlowMovingTraffic(self._config)
        interstate_broad_enough_pred = PredInterstateBroadEnough(self._config)
        on_shoulder_pred = PredOnShoulder(self._config)
        drives_leftmost_pred = PredDrivesLeftmost(self._config)
        in_single_lane_pred = PredSingleLane(self._config)
        in_leftmost_lane_pred = PredInLeftmostLane(self._config)
        drives_rightmost_pred = PredDrivesRightmost(self._config)
        in_rightmost_lane_pred = PredInRightmostLane(self._config)
        
        # Evaluate predicates
        ego_in_congestion = in_congestion_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_slow_traffic = in_slow_moving_traffic_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_broad_interstate = interstate_broad_enough_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_shoulder = on_shoulder_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_leftmost_lane = in_leftmost_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_single_lane = in_single_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_rightmost_lane = in_rightmost_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_drives_leftmost = drives_leftmost_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_drives_rightmost = drives_rightmost_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

        # Check if the vehicle is allowed to use the emergency lane
        if ego_in_congestion or ego_in_slow_traffic:
            
            # Check if the interstate is broad enough
            if ego_on_broad_interstate:
                if not ego_on_shoulder:   
                    if ego_in_leftmost_lane:
                        # If in the leftmost lane, ego must drive leftmost and be on a single lane
                        if ego_drives_leftmost and ego_on_single_lane:
                            return True, 0.0  # No violation
                    else:
                        # If not in the leftmost lane, ego must drive rightmost
                        if ego_drives_rightmost:
                            return True, 0.0  # No violation
            else:
                # If the interstate is not broad enough, ego must drive rightmost, in the rightmost lane, and on a single lane
                if ego_drives_rightmost and ego_in_rightmost_lane and ego_on_single_lane:
                    return True, 0.0  # No violation
        # If no conditions match, return False (indicating a violation) and the cost of violation TODO cost to be modelled as a function of the robustness
        in_left_most_lane_robusness = in_leftmost_lane_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
        return False, abs(in_left_most_lane_robusness)

    
    #--------------R_I_5--------------------    
    def check_consideration_of_entering_vehicles(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks the consideration of entering vehicles based on the given time step.

        Returns:
            Tuple[bool, Optional[float]]: A tuple containing a boolean value indicating whether the consideration is met,
            and an optional float value representing the unprocessed cost associated with violation.
        """
        # Initialize predicates
        on_main_carriageway_pred = PredOnMainCarriageway(self._config)
        in_front_of_pred = PredInFrontOf(self._config)
        on_access_ramp_pred = PredOnAccessRamp(self._config)
        on_main_carriageway_right_lane_pred = PredMainCarriageWayRightLane(self._config)
        
        ego_on_main_carriageway = on_main_carriageway_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        
        # Premise : ego_on_main_carriageway ^ (exists v in front of ego) ^ on_access_ramp ^ (exists v on main carriageway in future)
        if not ego_on_main_carriageway:
            return True, 0.0  # No violation, as the ego is not on the main carriageway

        # List of vehicles in front of ego
        vehicles_in_front_of_ego = [
            other_vehicle_id for other_vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if other_vehicle_id != self._ego_vehicle_id and in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
        ]

        if not vehicles_in_front_of_ego:
            return True, 0.0 # No violation, as there are no vehicles in front of the ego vehicle
        
        vehicles_on_access_ramp = [vehicle_id for vehicle_id in vehicles_in_front_of_ego if on_access_ramp_pred.evaluate_boolean(self._world, time_step, [vehicle_id])]
            
        if not vehicles_on_access_ramp:
            return True, 0.0 # No violation, as there are no vehicles on the access ramp in front of the ego vehicle
        
        vehicles_on_main_carriageway_f = [] # List of vehicles on the main carriageway in the future
        
        for other_vehicle_id in vehicles_on_access_ramp:
            other_vehicle = self._world.vehicle_by_id(other_vehicle_id)
            future_time = range(time_step, min(time_step + 5, len(other_vehicle.states_cr)))
            other_on_main_carriage_way_f = any(on_main_carriageway_pred.evaluate_boolean(self._world, t, [other_vehicle_id]) for t in future_time)
            
            if other_on_main_carriage_way_f:
                vehicles_on_main_carriageway_f.append(other_vehicle_id)

        if not vehicles_on_main_carriageway_f:
            return True, 0.0 # No violation, as the vehicles on the access ramp do not enter the main carriageway in the future
        
        # There is(are) vehicle(s) on the main carriageway in the future => ego must consider entering vehicles
        # Ego vehicle can be on the right lane of the main carriageway now but not in the future
        
        ego_on_main_carriageway_right_lane = on_main_carriageway_right_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        future_time = range(time_step, min(time_step + 5, len(self._ego_vehicle.states_cr)))
        ego_on_main_carriage_right_lane_f = any(on_main_carriageway_right_lane_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id]) for t in future_time)
        
        # Conclusion condition
        condition = ego_on_main_carriageway_right_lane or not ego_on_main_carriage_right_lane_f

        if not condition: # TODO cost to be modelled as a function of the robustness
            robustness = on_main_carriageway_right_lane_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
            return False, robustness  # Violation occurred
                
        return True, 0.0    # No violation
    
    
    #-----------------------INTERSECTION/URBAN RULES--------------------
    
    #--------------R_IN_1--------------------
    def check_stopping_at_stop_sign(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if the ego vehicle is stopping at a stop sign.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean value indicating if the ego vehicle is stopping at stop sign,
            and a float value representing the cost of violation.
        """
        # Initialize predicates
        stop_line_in_front_pred = PredStopLineInFront(self._config)
        in_stand_still_pred = PredInStandStill(self._config)
        # Evaluate predicates
        stop_line_in_front = stop_line_in_front_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        stop_line_in_front_p = stop_line_in_front_pred.evaluate_boolean(self._world, max(time_step - 1, 0), [self._ego_vehicle_id])
        
        # If the stop line was in front in the past and is still in front, check the standstill condition
        if stop_line_in_front_p and stop_line_in_front: # TODO implementation to be corrected
            for t in range(max(0, time_step - 3), time_step + 1):
                if not (stop_line_in_front_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id]) and 
                        in_stand_still_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id])):
                    
                    return False, 10.0  # TODO: Define the cost of violation
        
        # No violation detected
        return True, 0.0
    
    
    #--------------R_IN_2--------------------    
    def check_stopping_at_red_light(self, time_step: int) -> Tuple[bool, float]:
        """
        Checks if the ego vehicle is stopping at a red light.

        Returns:
            Tuple[bool, float]: A tuple containing a boolean value indicating if the ego vehicle is stopping at a red light,
            and a float value representing the cost of violation.
        """
        # Initialize predicates
        traffic_light_red_pred = PredTrafficLightRed(self._config)
        stop_line_in_front_pred = PredStopLineInFront(self._config)
        on_intersection_pred = PredInIntersection(self._config)
        on_right_turn_pred = PredOnRightTurn(self._config)
        in_stand_still_pred = PredInStandStill(self._config)
        # Evaluate predicates
        traffic_light_red = traffic_light_red_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        stop_line_in_front = stop_line_in_front_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_intersection = on_intersection_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_right_turn = on_right_turn_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_stand_still = in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        
        if traffic_light_red and stop_line_in_front and not ego_on_intersection and not ego_on_right_turn:
            if not ego_in_stand_still:
                return False, 10.0  # TODO: Define the cost of violation
        # No violation detected
        return True, 0.0