import os

from typing import Tuple
from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap
from vehiclemodels import parameters_vehicle2


from commonroad.scenario.state import CustomState
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle
from crmonitor.common.world import World, RoadNetwork, Vehicle, CurvilinearStateManager

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







class RuleCheckerBase:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory, config_path: str = None) -> None:
                 
        self._config = self._load_configuration(config_path)
        self._road_network = RoadNetwork(scenario.lanelet_network, self._config.get("road_network_param")) 
        self._ego_vehicle_id = 0
        # Initialize Vehicles                        
        self._ego_vehicle = create_ego_vehicle_from_trajectory(self._road_network, ego_trajectory, self._config)
        
        self._other_vehicles = [create_other_vehicle_from_obstacle(obstacle, self._road_network, self._config)
                                for obstacle in scenario.dynamic_obstacles
                                if obstacle.obstacle_id != self._ego_vehicle_id]
                 
        self._vehicles = [self._ego_vehicle] + self._other_vehicles 
        
        self._update_vehicle_parameters()
            
        # self.v_id = [vehicle.id for vehicle in self._vehicles]
        # print(f"Ego ID: {self._ego_vehicle_id}")
        # for vehicle in self._vehicles:
        #     print(f"Vehicle {vehicle.id} : StartTime: {min(vehicle.states_cr.keys())} : EndTime: {max(vehicle.states_cr.keys())}")
      
        # Update vehicle parameters
        self._update_vehicle_parameters()
   
    def _load_configuration(self, config_path: str) -> dict:
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
    
    def _update_vehicle_parameters(self) -> None:
        for vehicle in self._other_vehicles:
            if vehicle.id == self._ego_vehicle_id:
                continue  # Skip the ego vehicle
            vehicle.vehicle_param.update(self._config.get("other_vehicles_param", {}))
        
        self._ego_vehicle.vehicle_param.update(self._config.get("ego_vehicle_param", {}))


#-----------------------R_G_1-Rule--------------------------------
class SafeDistanceChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._in_same_lane_pred = PredInSameLane(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._safe_distance_pred = PredSafeDistPrec(self._config)
        self._cut_in_pred = PredCutIn(self._config)
        self._preceding_pred = PredPreceding(self._config)

    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (self._world.vehicle_by_id(vehicle_id).rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None and
                self._world.vehicle_by_id(vehicle_id).get_lane(time_step) is not None and
                self._world.vehicle_by_id(vehicle_id).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step)) is not None)
        ]
        # Vehicles in front of ego vehicle
        vehicles_in_front_of_ego = [
            vehicle_id for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id and
               self._in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        #print(f"Time {time_step}: Vehicles in front of ego: {vehicles_in_front_of_ego}")
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        # If no vehicles are found in the same lane and in front of ego, return safe
        if not vehicles_in_front_of_and_in_same_lane_as_ego:
            return True, 0.0
        # Sort vehicles in front by robustness of "in_front_of" predicate
        sorted_vehicles = sorted(
            vehicles_in_front_of_and_in_same_lane_as_ego,
            key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        )
        preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front
        preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id)
        
        # Evaluate whether safe distance is kept
        cut_in_happened_o = False
        cut_in_happened_p = False
        t_c = 3  # According to Maierhofer et al.
        previous_start_time = max(0, time_step - t_c)
        valid_time_steps = range(previous_start_time, time_step) 
        if previous_start_time and time_step in preceding_vehicle.states_cr:
            cut_in_happened_p = self._cut_in_pred.evaluate_boolean(self._world, max(0, time_step - 1), [preceding_vehicle_id, self._ego_vehicle_id]) # Check if cut-in happened at previous time step
            cut_in_happened_o = any([self._cut_in_pred.evaluate_boolean(self._world, t, [preceding_vehicle_id, self._ego_vehicle_id]) for t in valid_time_steps]) # Check if cut-in happened at some time step in the past                                  
        # evaluate if safe distance is kept  
        safe_distance_kept = self._safe_distance_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])  
                                                              
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept: # Temporal Logic resolution in CNF
          #  self.logger.info(f"Safe Distance Maintained following {other_vehicle_id}, maybe violated only due to a cut-in at previous time: {time_step}")
            return True, 0.0  # safe distance is maintained given a preceding vehicle
        
        else: # Safe distance is violated, when cut_in_happened_o and cut_in_happened_p and safe_distance_kept are False
            safe_distance_robusness = self._safe_distance_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]) # returns (actual_distance - safe_distance) => -ve value for non-compliance
            return False, abs(safe_distance_robusness)


#-----------------------R_G_2-Rule--------------------------------
class UnnecessaryBrakingChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        # for "Preceding" predicate use 'same lane' and 'in front of' predicates
        self._in_same_lane_pred = PredInSameLane(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._safe_distance_pred = PredSafeDistPrec(self._config)
        self._abrupt_braking_pred = PredAbruptBreaking(self._config)
        self._rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)

    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (self._world.vehicle_by_id(vehicle_id).rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None and
                self._world.vehicle_by_id(vehicle_id).get_lane(time_step) is not None and
                self._world.vehicle_by_id(vehicle_id).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step)) is not None)
        ]
        
        vehicles_in_front_of_ego = [
            vehicle_id for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id and
               self._in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]        
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        
        if len(vehicles_in_front_of_and_in_same_lane_as_ego) == 0: #  Case 1 No preceding vehicle, check only abrupt braking
            print(f"EgoAcceleration = {self._ego_vehicle.states_cr[time_step].acceleration}")
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            
            if not braked_abruptly:
                return True, 0.0   
                
            else:
                robustness = self._abrupt_braking_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                return False, abs(robustness)     
    
        else: # Case 2 : There is a preceding vehicle, check both abrupt braking and relative abrupt braking
            sorted_vehicles = sorted(
                vehicles_in_front_of_and_in_same_lane_as_ego,
                key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
            )
            preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front         
            # Using robustness as the cost of violation, TODO cost to be modelled as a function of the abruptness of braking
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            braked_abruptly_rel = self._rel_abrupt_braking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])
            safe_distance_kept = self._safe_distance_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])
            # Using CNF resolution of the temporal logic 
            condition_1 = (not braked_abruptly or preceding_vehicle_id)
            condition_2 = (not braked_abruptly or not safe_distance_kept or not braked_abruptly_rel)
            
            if condition_1 and condition_2: # True => abrupt relative braking
                robustness = self._rel_abrupt_braking_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id])
                return False, abs(robustness)
            else:
                # No relative abrupt braking
                return True, 0.0

    
#-----------------------R_G_3-Rule--------------------------------
class MaximumSpeedLimitChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._lane_speed_limit_pred = PredLaneSpeedLimit(self._config)
        self._brake_speed_limit_pred = PredBrSpeedLimit(self._config)
        self._fov_speed_limit_pred = PredFovSpeedLimit(self._config)
        self._type_speed_limit_pred = PredTypeSpeedLimit(self._config)

    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        lane_speed_limit = self._lane_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        brake_speed_limit = self._brake_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        fov_speed_limit = self._fov_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])
        type_speed_limit = self._type_speed_limit_pred.get_speed_limit(self._world, time_step, [self._ego_vehicle_id])

        lane_speed_limit = float('inf') if lane_speed_limit is None else lane_speed_limit
        brake_speed_limit = float('inf') if brake_speed_limit is None else brake_speed_limit
        fov_speed_limit = float('inf') if fov_speed_limit is None else fov_speed_limit
        type_speed_limit = float('inf') if type_speed_limit is None else type_speed_limit

        ego_velocity = self._ego_vehicle.states_cr[time_step].velocity
        violation = max(0, ego_velocity - min(lane_speed_limit, brake_speed_limit, fov_speed_limit, type_speed_limit))
        return violation == 0, violation


#-----------------------R_G_4-Rule--------------------------------
class TrafficFlowChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._slow_leading_vehicle_pred = PredSlowLeadingVehicle(self._config)
        self._preserves_traffic_flow_pred = PredPreservesTrafficFlow(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._in_same_lane_pred = PredInSameLane(self._config)
              
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (self._world.vehicle_by_id(vehicle_id).rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None and
                self._world.vehicle_by_id(vehicle_id).get_lane(time_step) is not None and
                self._world.vehicle_by_id(vehicle_id).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step)) is not None)
        ]
        
        vehicles_in_front_of_ego = [
            vehicle_id for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id and
               self._in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]        
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        
        if len(vehicles_in_front_of_and_in_same_lane_as_ego) == 0:
            ego_preserves_traffic_flow = self._preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step,[self._ego_vehicle_id])  
                                                                                        
            if ego_preserves_traffic_flow:
                return True, 0.0
            
            else: # TODO cost to be modelled as a function of the robustness of the traffic flow preservation
                robustness = self._preserves_traffic_flow_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                                                                             
                return False, abs(robustness)
        else:
            # There is a leading vehicle, check if there is a slow leading vehicle
            try: 
                there_is_a_slow_leading_vehicle = self._slow_leading_vehicle_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])  # Line doesnt work for some scenarios                                                                         
                ego_preserves_traffic_flow = self._preserves_traffic_flow_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id]) 
                                                                                     
                if there_is_a_slow_leading_vehicle or ego_preserves_traffic_flow: # Resolution into CNF
                    return True, 0.0 
                else:
                    # There is a violation 
                    robustness = self._preserves_traffic_flow_pred.evaluate_robustness(self._world, time_step,[self._ego_vehicle_id])    
                    return False, abs(robustness)     
                
            except TypeError as e:
               # self._logger.info(f"Error processing vehicle for traffic_flow check @ time {time_step} returning True--- : {e}")
                return True, 0.0


#-----------------------R_I_1-Rule--------------------------------
class NoStoppingChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)       
        self._in_stand_still_pred = PredInStandStill(self._config)
        self._standing_leading_vehicle_pred = PredExistStandingLeadingVehicle(self._config)
        self._in_congestion_pred = PredInCongestion(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._in_same_lane_pred = PredInSameLane(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (self._world.vehicle_by_id(vehicle_id).rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None and
                self._world.vehicle_by_id(vehicle_id).get_lane(time_step) is not None and
                self._world.vehicle_by_id(vehicle_id).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step)) is not None)
        ]
        
        vehicles_in_front_of_ego = [
            vehicle_id for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id and
               self._in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]        
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, vehicle_id])
        ]
        
        if len(vehicles_in_front_of_and_in_same_lane_as_ego) == 0:
            ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

            if ego_in_stand_still: # TODO cost to be modelled as a function of the robustness of the stopping behavior
                robustness = self._in_stand_still_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                return False, robustness  # Unnecessary stopping
            else:
                return True, 0.0  # No stopping detected

        else:  # There is a leading vehicle, perform extensive check
            try:
                ego_in_congestion = self._in_congestion_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
                there_is_a_standing_leading_vehicle = self._standing_leading_vehicle_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
                ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])

                # Temporal Logic Resolution into CNF
                condition_1 = (not ego_in_stand_still or ego_in_congestion)  # If ego is in standstill, ensure it's in congestion
                condition_2 = (not ego_in_stand_still or not there_is_a_standing_leading_vehicle)  # If in standstill, check if leading vehicle is also standing still

                if not (condition_1 and condition_2):  # If either condition fails, there is unnecessary stopping
                    robustness = self._in_stand_still_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
                    return False, robustness
                else:
                    return True, 0.0  # No unnecessary stopping detected
            except Exception as ex:
               # self._logger.info(f"Error processing vehicle for no_stopping check @ time {time_step}: returning True {ex}")
                return True, 0.0 # Assuming no violation
    
        
#-----------------------R_I_2-Rule--------------------------------
class DrivingFasterThanLeftTrafficChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._left_of_pred = PredLeftOf(self._config)
        self._drives_faster_pred = PredDrivesFaster(self._config)
        self._in_congestion_pred = PredInCongestion(self._config)
        self._in_slow_moving_traffic_pred = PredInSlowMovingTraffic(self._config)
        self._in_queue_pred = PredInQueueOfVehicles(self._config)
        self._drives_slightly_higher_speed_pred = PredDrivesWithSlightlyHigherSpeed(self._config)
        self._right_of_broad_lane_marking_pred = PredRightOfBroadLaneMarking(self._config)
        self._left_of_broad_lane_marking_pred = PredLeftOfBroadLaneMarking(self._config)
        self._on_access_ramp_pred = PredOnAccessRamp(self._config)
        self._on_main_carriageway_pred = PredOnMainCarriageway(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        # List all vehicles on the left of the ego vehicle at the this time step
        valid_vehicle_ids = [ 
            vehicle_id for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (self._world.vehicle_by_id(vehicle_id).rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None and
                self._world.vehicle_by_id(vehicle_id).get_lane(time_step) is not None and
                self._world.vehicle_by_id(vehicle_id).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step)) is not None)
        ]
        
        vehicles_on_the_left_of_ego = [
            other_vehicle_id for other_vehicle_id in valid_vehicle_ids
            if other_vehicle_id != self._ego_vehicle_id and self._left_of_pred.evaluate_boolean(self._world, time_step, [ self._ego_vehicle_id, other_vehicle_id])
        ]

        if not vehicles_on_the_left_of_ego:  # No vehicles on the left means => trivially no violation
            return True, 0.0
        # Check if the ego vehicle is driving faster than any vehicle on its left, returns the first violation
        for other_vehicle_id in vehicles_on_the_left_of_ego:
            ego_drives_faster = self._drives_faster_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            if not ego_drives_faster:  # Ego is not driving faster than this vehicle, continue checking others
                continue
            # Check if the conditions of safe overtaking are met
            other_in_queue = self._in_queue_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            other_in_congestion = self._in_congestion_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            other_in_slow_traffic = self._in_slow_moving_traffic_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            ego_slightly_faster = self._drives_slightly_higher_speed_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            ego_right_broad_marking = self._right_of_broad_lane_marking_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            other_left_broad_marking = self._left_of_broad_lane_marking_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            ego_on_access_ramp = self._on_access_ramp_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
            other_on_main_carriageway = self._on_main_carriageway_pred.evaluate_boolean(self._world, time_step, [other_vehicle_id])
            
            # According to the paper, the following conditions should be met if the ego vehicle is driving faster than the traffic on its left
            condition_1 = (other_in_queue or other_in_congestion or other_in_slow_traffic) and ego_slightly_faster
            condition_2 = ego_right_broad_marking and other_left_broad_marking
            condition_3 = ego_on_access_ramp and other_on_main_carriageway and not (other_in_queue or other_in_congestion or other_in_slow_traffic)

            # If any of the conditions are met, there is no violation
            if condition_1 or condition_2 or condition_3:
                return True, 0.0  # No violation

            # If none of the conditions are met, calculate the speed difference as a violation TODO cost to be modelled as a function of the speed difference
            speed_difference = self._drives_faster_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
            return False, speed_difference  # Violation occurred

        # If no vehicles cause a violation, return no violation
        return True, 0.0
    
                
#-----------------------R_I_3-Rule--------------------------------  
class ReversingAndTurningChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._reverses_pred = PredReverses(self._config)
        self._makes_u_turn_pred = PredMakesUTurn(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        ego_makes_u_turn = self._makes_u_turn_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_reverses = self._reverses_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        
        if not ego_makes_u_turn and not ego_reverses:
            return True, 0.0
        else:
            # Return the robustness as the cost of violation # TODO cost to be modelled as a function of the robustness
            robustness = self._makes_u_turn_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id]) + \
                         self._reverses_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
            return False, abs(robustness)
       
 
#-----------------------R_I_4-Rule--------------------------------
class EmergencyLaneChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._in_congestion_pred = PredInCongestion(self._config)
        self._in_slow_moving_traffic_pred = PredInSlowMovingTraffic(self._config)
        self._interstate_broad_enough_pred = PredInterstateBroadEnough(self._config)
        self._on_shoulder_pred = PredOnShoulder(self._config)
        self._drives_leftmost_pred = PredDrivesLeftmost(self._config)
        self._in_single_lane_pred = PredSingleLane(self._config)
        self._in_leftmost_lane_pred = PredInLeftmostLane(self._config)
        self._drives_rightmost_pred = PredDrivesRightmost(self._config)
        self._in_rightmost_lane_pred = PredInRightmostLane(self._config)   
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:   
        ego_in_congestion = self._in_congestion_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_slow_traffic = self._in_slow_moving_traffic_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_broad_interstate = self._interstate_broad_enough_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_shoulder = self._on_shoulder_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_leftmost_lane = self._in_leftmost_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_single_lane = self._in_single_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_rightmost_lane = self._in_rightmost_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_drives_leftmost = self._drives_leftmost_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_drives_rightmost = self._drives_rightmost_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
             
        if ego_in_congestion or ego_in_slow_traffic:
            if ego_on_broad_interstate:
                if not ego_on_shoulder:   
                    if ego_in_leftmost_lane:
                        if ego_drives_leftmost and ego_on_single_lane:
                            return True, 0.0  # No violation
                    else:
                        if ego_drives_rightmost:
                            return True, 0.0  # No violation
            else:
                # If the interstate is not broad enough, ego must drive rightmost, in the rightmost lane, and on a single lane
                if ego_drives_rightmost and ego_in_rightmost_lane and ego_on_single_lane:
                    return True, 0.0  # No violation
        # If no conditions match, return False (indicating a violation) and the cost of violation TODO cost to be modelled as a function of the robustness
        in_left_most_lane_robusness = self._in_leftmost_lane_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
        return False, 5 # TODO cost to be modelled as a function of the robustness
    

#-----------------------R_I_5-Rule--------------------------------
class ConsiderEnteringVehicleChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._on_main_carriageway_pred = PredOnMainCarriageway(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._on_access_ramp_pred = PredOnAccessRamp(self._config)
        self._on_main_carriageway_right_lane_pred = PredMainCarriageWayRightLane(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        ego_on_main_carriageway = self._on_main_carriageway_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        # Premise : ego_on_main_carriageway ^ (exists v in front of ego) ^ on_access_ramp ^ (exists v on main carriageway in future)
        if not ego_on_main_carriageway:
            return True, 0.0  # No violation, as the ego is not on the main carriageway

        # List of vehicles in front of ego
        vehicles_in_front_of_ego = [
            other_vehicle_id for other_vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if other_vehicle_id != self._ego_vehicle_id and self._in_front_of_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id, other_vehicle_id])
        ]

        if not vehicles_in_front_of_ego:
            return True, 0.0 # No violation, as there are no vehicles in front of the ego vehicle
        
        vehicles_on_access_ramp = [vehicle_id for vehicle_id in vehicles_in_front_of_ego if self._on_access_ramp_pred.evaluate_boolean(self._world, time_step, [vehicle_id])]
            
        if not vehicles_on_access_ramp:
            return True, 0.0 # No violation, as there are no vehicles on the access ramp in front of the ego vehicle
        
        vehicles_on_main_carriageway_f = [] # List of vehicles on the main carriageway in the future
        
        for other_vehicle_id in vehicles_on_access_ramp:
            other_vehicle = self._world.vehicle_by_id(other_vehicle_id)
            future_time = range(time_step, min(time_step + 5, len(other_vehicle.states_cr)))
            other_on_main_carriage_way_f = any(self._on_main_carriageway_pred.evaluate_boolean(self._world, t, [other_vehicle_id]) for t in future_time)
            
            if other_on_main_carriage_way_f:
                vehicles_on_main_carriageway_f.append(other_vehicle_id)

        if not vehicles_on_main_carriageway_f:
            return True, 0.0 # No violation, as the vehicles on the access ramp do not enter the main carriageway in the future
        
        # There is(are) vehicle(s) on the main carriageway in the future => ego must consider entering vehicles
        # Ego vehicle can be on the right lane of the main carriageway now but not in the future
        
        ego_on_main_carriageway_right_lane = self._on_main_carriageway_right_lane_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        future_time = range(time_step, min(time_step + 5, len(self._ego_vehicle.states_cr)))
        ego_on_main_carriage_right_lane_f = any(self._on_main_carriageway_right_lane_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id]) for t in future_time)
        
        # Conclusion condition
        condition = ego_on_main_carriageway_right_lane or not ego_on_main_carriage_right_lane_f

        if not condition: # TODO cost to be modelled as a function of the robustness
            robustness = self._on_main_carriageway_right_lane_pred.evaluate_robustness(self._world, time_step, [self._ego_vehicle_id])
            return False, robustness  # Violation occurred
              
        return True, 0.0    # No violation   
  

#-----------------------R_IN_1-Rule--------------------------------   
class StopAtStopSignChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._stop_line_in_front_pred = PredStopLineInFront(self._config)
        self._in_stand_still_pred = PredInStandStill(self._config) 
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        stop_line_in_front = self._stop_line_in_front_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        stop_line_in_front_p = self._stop_line_in_front_pred.evaluate_boolean(self._world, max(time_step - 1, 0), [self._ego_vehicle_id])
        
        # If the stop line was in front in the past and is still in front, check the standstill condition
        if stop_line_in_front_p and stop_line_in_front: # TODO implementation to be corrected
            for t in range(max(0, time_step - 3), time_step + 1):
                if not (self._stop_line_in_front_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id]) and 
                        self._in_stand_still_pred.evaluate_boolean(self._world, t, [self._ego_vehicle_id])):
                    return False, 10.0  # TODO: Define the cost of violation
        # No violation detected
        return True, 0.0


#-----------------------R_IN_2-Rule--------------------------------
class StopAtRedLightChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network, scenario)
        self._traffic_light_red_pred = PredTrafficLightRed(self._config)
        self._stop_line_in_front_pred = PredStopLineInFront(self._config)
        self._on_intersection_pred = PredInIntersection(self._config)
        self._on_right_turn_pred = PredOnRightTurn(self._config)
        self._in_stand_still_pred = PredInStandStill(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        traffic_light_red = self._traffic_light_red_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        stop_line_in_front = self._stop_line_in_front_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_intersection = self._on_intersection_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_on_right_turn = self._on_right_turn_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(self._world, time_step, [self._ego_vehicle_id])
        
        if traffic_light_red and stop_line_in_front and not ego_on_intersection and not ego_on_right_turn:
            if not ego_in_stand_still:
                return False, 10.0  # 
        # No violation detected
        return True, 0.0         

 
    

def create_other_vehicle_from_obstacle(obstacle: DynamicObstacle, road_network: RoadNetwork, config) -> Vehicle:
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
            acceleration = (state.velocity - prev_state.velocity) / 0.1  # Assuming time_step difference is 0.1
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



def create_ego_vehicle_from_trajectory(road_network: RoadNetwork, ego_trajectory : Trajectory, config) -> Vehicle:
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
            acceleration = (state.velocity - prev_state.velocity) / 0.1  # change according to scenario.dt , using 0.1 for now 
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
        shape=Rectangle(width=parameters_vehicle2.parameters_vehicle2().w, length=parameters_vehicle2.parameters_vehicle2().l),
        states_cr=states_cr,
        signal_series=None,
        ccosy_cache=CurvilinearStateManager(road_network),
        lanelet_assignment=lanelet_assignment,
    )
    return ego_vehicle