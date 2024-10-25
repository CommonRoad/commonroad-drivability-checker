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
    def __init__(
        self, scenario: Scenario, ego_trajectory: Trajectory, config_path: str = None
    ) -> None:
           
        self._config = self._load_configuration(config_path)
        self._road_network = RoadNetwork(
            scenario.lanelet_network, self._config.get("road_network_param")
        )
        
        self._ego_vehicle_id = 0 # Ego vehicle ID is always 0
        self._ego_vehicle = create_ego_vehicle_from_trajectory(
            ego_trajectory, self._road_network,  self._config
        )
        
        self._other_vehicles = [
            create_other_vehicle_from_obstacle(
                obstacle, self._road_network, self._config
                )
                for obstacle in scenario.dynamic_obstacles
                if obstacle.obstacle_id != self._ego_vehicle_id
        ]
        self._vehicles = [self._ego_vehicle] + self._other_vehicles # All vehicles in the scenario      
        self._update_vehicle_parameters() # Update vehicle parameters
    
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
            vehicle.vehicle_param.update(self._config.get("other_vehicles_param", {})) # Update other vehicle parameters
        self._ego_vehicle.vehicle_param.update(self._config.get("ego_vehicle_param", {})) # Update ego vehicle parameters

#-----------------------R_G_1-Rule--------------------------------
class SafeDistanceChecker(RuleCheckerBase):
    def __init__(self, scenario : Scenario, ego_trajectory):
        super().__init__(scenario,  ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._in_same_lane_pred = PredInSameLane(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._safe_distance_pred = PredSafeDistPrec(self._config)
        self._cut_in_pred = PredCutIn(self._config)
        self._preceding_pred = PredPreceding(self._config)

    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        
        valid_vehicle_ids = [
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
            )
        ]
        
        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ] 
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        if not vehicles_in_front_of_and_in_same_lane_as_ego: # No vehicles in front of ego in the same lane
            return True, 0.0

        sorted_vehicles = sorted(
            vehicles_in_front_of_and_in_same_lane_as_ego,
            key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        )
        
        preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front
        preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id)
        
        # Cut_in check deactiated for now because it is not working as expected(issue with the predicate)
        cut_in_happened_o = False
        cut_in_happened_p = False
        t_c = 3  
        # previous_start_time = max(0, time_step - t_c)
        # valid_time_steps = range(previous_start_time, time_step) 
        # if previous_start_time and time_step in preceding_vehicle.states_cr:
        #     cut_in_happened_p = self._cut_in_pred.evaluate_boolean(self._world, max(0, time_step - 1), [preceding_vehicle_id, self._ego_vehicle_id]) # Check if cut-in happened at previous time step
        #     cut_in_happened_o = any([self._cut_in_pred.evaluate_boolean(self._world, t, [preceding_vehicle_id, self._ego_vehicle_id]) for t in valid_time_steps]) # Check if cut-in happened at some time step in the past                                  
        # evaluate if safe distance is kept  
        safe_distance_kept = self._safe_distance_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]
        )  
                                                              
        if cut_in_happened_o or cut_in_happened_p or safe_distance_kept: # Temporal Logic resolution in CNF
            return True, 0.0  # safe distance is maintained given a preceding vehicle
        
        else: # Safe distance is violated, when cut_in_happened_o and cut_in_happened_p and safe_distance_kept are all False
            safe_distance_robusness = self._safe_distance_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]
            ) # returns (actual_distance - safe_distance) => -ve value for non-compliance
            
            return False, abs(safe_distance_robusness)


#-----------------------R_G_2-Rule--------------------------------
class UnnecessaryBrakingChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._in_same_lane_pred = PredInSameLane(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._safe_distance_pred = PredSafeDistPrec(self._config)
        self._abrupt_braking_pred = PredAbruptBreaking(self._config)
        self._rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)

    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        
        valid_vehicle_ids = [
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
            )
        ]

        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        if not vehicles_in_front_of_and_in_same_lane_as_ego: #  Case 1 No preceding vehicle, check only abrupt braking
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            
            if not braked_abruptly:
                return True, 0.0   
               
            else:
                robustness = self._abrupt_braking_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id]
                )
                
                return False, abs(robustness)     
    
        else: # Case 2 : There is a preceding vehicle, check both abrupt braking and relative abrupt braking
            sorted_vehicles = sorted(
                vehicles_in_front_of_and_in_same_lane_as_ego,
                key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id, vehicle_id]
                )
            )
            
            preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front         
            # Using robustness as the cost of violation
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            
            braked_abruptly_relative = self._rel_abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]
            )
            
            safe_distance_kept = self._safe_distance_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]
            )
            # Using CNF resolution of the temporal logic 
            condition = (not braked_abruptly
                         or not safe_distance_kept
                         or not braked_abruptly_relative) 
 
            if condition: # True => abrupt braking
                robustness = self._rel_abrupt_braking_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id, preceding_vehicle_id]
                )
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
        
        lane_speed_limit_kept = self._lane_speed_limit_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        brake_speed_limit_kept = self._brake_speed_limit_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        fov_speed_limit_kept = self._fov_speed_limit_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        type_speed_limit_kept = self._type_speed_limit_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        
        condition = (lane_speed_limit_kept
                     and brake_speed_limit_kept
                     and fov_speed_limit_kept
                     and type_speed_limit_kept)
        
        if condition:
            return True, 0.0
        
        else: # At least one speed limit is violated
            bool_rob_set = {
                    lane_speed_limit_kept :  self._lane_speed_limit_pred.evaluate_robustness(
                                                    self._world, time_step, [self._ego_vehicle_id]
                                            ),
                    brake_speed_limit_kept : self._brake_speed_limit_pred.evaluate_robustness(
                                                    self._world, time_step, [self._ego_vehicle_id]
                                            ),
                    fov_speed_limit_kept : self._fov_speed_limit_pred.evaluate_robustness(
                                                    self._world, time_step, [self._ego_vehicle_id]
                                            ),
                    type_speed_limit_kept : self._type_speed_limit_pred.evaluate_robustness(
                                                    self._world, time_step, [self._ego_vehicle_id]
                                            )
            }
            
            rob_list = []
            
            for speed_limit_kept, rob in bool_rob_set.items():
                if not speed_limit_kept:
                    rob_list.append(abs(rob))
            
            return False , sum(rob_list)
                    
 
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
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
            )
        ]

        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        if not vehicles_in_front_of_and_in_same_lane_as_ego:
            ego_preserves_traffic_flow = self._preserves_traffic_flow_pred.evaluate_boolean(
                self._world, time_step,[self._ego_vehicle_id]
            )  
                                                                                        
            if ego_preserves_traffic_flow:
                return True, 0.0
            
            else: 
                robustness = self._preserves_traffic_flow_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id]
                )
                                                                             
                return False, abs(robustness)
        else:
            there_is_a_slow_leading_vehicle = self._slow_leading_vehicle_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )                                                                     
            ego_preserves_traffic_flow = self._preserves_traffic_flow_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            ) 
                                                                                    
            if there_is_a_slow_leading_vehicle or ego_preserves_traffic_flow: # Resolution into CNF
                return True, 0.0 
            
            else: # slow leading vehicle is not present and ego does not preserve traffic flow
                robustness = self._preserves_traffic_flow_pred.evaluate_robustness(
                                    self._world, time_step, [self._ego_vehicle_id]
                )
                
                return False, abs(robustness) # Assuming no violation
                

#-----------------------R_I_1-Rule--------------------------------
class StoppingChecker(RuleCheckerBase):
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
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
            )
        ]

        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self._ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]  

        if not vehicles_in_front_of_and_in_same_lane_as_ego:
            ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )

            if not ego_in_stand_still:
                return True, 0.0
            
            else:
                robustness = self._in_stand_still_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id]
                )
                return False, robustness  # Stopping detected

        else: # There is a leading vehicle, perform extensive check
            ego_in_congestion = self._in_congestion_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            there_is_a_standing_leading_vehicle = self._standing_leading_vehicle_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            
            if not ego_in_stand_still:
                return True, 0.0
            
            else:
                condition = ego_in_congestion or there_is_a_standing_leading_vehicle
                
                if condition:
                    return True, 0.0
                
                else:
                    robustness = self._in_stand_still_pred.evaluate_robustness(
                                        self._world, time_step, [self._ego_vehicle_id]
                    )
        
                    return False, robustness
                
                
                     
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
        
        valid_vehicle_ids = [
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
                
            )
        ]

        vehicles_on_the_left_of_ego = [
            vehicle_id 
            for vehicle_id in valid_vehicle_ids
            if  vehicle_id != self._ego_vehicle_id
            and self._left_of_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id, self._ego_vehicle_id]
            )
        ]

        if not vehicles_on_the_left_of_ego:  # No vehicles on the left => trivially no violation
            return True, 0.0

        vehicles_ego_drives_faster_than = [
            vehicle_id
            for vehicle_id in vehicles_on_the_left_of_ego
            if self._drives_faster_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        
        if not vehicles_ego_drives_faster_than:  # ego is not driving faster than any vehicle on its left
            return True, 0.0  # No violation
        
        for vehicle_id in vehicles_ego_drives_faster_than: # check for the first violation
            
            other_in_queue = self._in_queue_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id]
            )
            other_in_congestion = self._in_congestion_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id]
            )
            other_in_slow_traffic = self._in_slow_moving_traffic_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id]
            )
            ego_slightly_faster = self._drives_slightly_higher_speed_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
            ego_right_broad_marking = self._right_of_broad_lane_marking_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            other_left_broad_marking = self._left_of_broad_lane_marking_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id]
            )
            ego_on_access_ramp = self._on_access_ramp_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id]
            )
            other_on_main_carriageway = self._on_main_carriageway_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id]
            )

            condition_1 = other_in_queue or other_in_congestion or other_in_slow_traffic
            condition_2 = (condition_1) and ego_slightly_faster
            condition_3 = ego_right_broad_marking and other_left_broad_marking
            condition_4 = ego_on_access_ramp and other_on_main_carriageway and not (condition_1)

            if condition_2 or condition_3 or condition_4:
                return True, 0.0  # No violation
            
            robustness = self._drives_faster_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
            
            return False, robustness  # Violation occurred
        
                
#-----------------------R_I_3-Rule--------------------------------  
class ReversingAndTurningChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._reverses_pred = PredReverses(self._config)
        self._makes_u_turn_pred = PredMakesUTurn(self._config)
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        
        ego_makes_u_turn = self._makes_u_turn_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        
        ego_reverses = self._reverses_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        
        if not ego_makes_u_turn and not ego_reverses:
            return True, 0.0
        
        # At least one of the predicates is true
        bool_rob_set = {
            ego_makes_u_turn : self._makes_u_turn_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id]
            ),
            ego_reverses : self._reverses_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id]
            )
        }
        
        rob_list = []
        
        for u_turn_or_reverse, rob in bool_rob_set.items():
            if u_turn_or_reverse:
                rob_list.append(abs(rob))
        
        return False, sum(rob_list)


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
         
        ego_in_congestion = self._in_congestion_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_in_slow_traffic = self._in_slow_moving_traffic_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_on_broad_interstate = self._interstate_broad_enough_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_on_shoulder = self._on_shoulder_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_in_leftmost_lane = self._in_leftmost_lane_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_on_single_lane = self._in_single_lane_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_in_rightmost_lane = self._in_rightmost_lane_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_drives_leftmost = self._drives_leftmost_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_drives_rightmost = self._drives_rightmost_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
         
        if not (ego_in_congestion or ego_in_slow_traffic):
                return True, 0.0  # Rule trivially maintained    
 
        # CASE 1: Ego is on a BROAD_INTERSTATE
        # Ego should not be on the SHOULDER and should be on LEFTMOST_LANE
        # On LEFTMOST_LANE means DRIVING_LEFTMOST and on a SINGLE_LANE
        # Not on LEFTMOST_LANE means DRIVING_RIGHTMOST
        if ego_on_broad_interstate:
            condition = not ego_on_shoulder and ego_drives_leftmost and ego_in_leftmost_lane
            
            if condition:
                return True, 0.0
            
            else: # penalize for being on the shoulder or not driving leftmost
                robustness = self._on_shoulder_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id]
                )
                return False, abs(robustness)
            
        # CASE 2: Ego is NOT on a BROADER_INTERSTATE
        # Ego should DRIVE_RIGHTMOST and should be on RIGHTMOST_LANE AND on SINGLE_LANE
        # Ego can also be on the SHOULDER
        else:
            condition = (ego_drives_rightmost and ego_in_rightmost_lane and ego_on_single_lane) or ego_on_shoulder
            
            if condition:
                return True, 0.0
            
            else:
                # Not rightmost driving => leftmost driving, hence penalize 
                robustness = self._drives_leftmost_pred.evaluate_robustness(
                    self._world, time_step, [self._ego_vehicle_id]
                )
                return False, abs(robustness)    
        
        
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
        
        ego_on_main_carriageway = self._on_main_carriageway_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        # PREMISE : ego_on_main_carriageway ^ (exists v in front of ego) ^ on_access_ramp ^ (exists v on main carriageway in future)
        if not ego_on_main_carriageway:
            return True, 0.0  # No violation, as the ego is not on the main carriageway
        
        valid_vehicle_ids = [
            vehicle_id
            for vehicle_id in self._world.vehicle_ids_for_time_step(time_step)
            if (
                self._world.vehicle_by_id(vehicle_id).rear_s(
                    time_step, self._ego_vehicle.get_lane(time_step)
                )
                is not None
                and self._world.vehicle_by_id(vehicle_id).get_lane(time_step)
                is not None
                and self._world.vehicle_by_id(
                    vehicle_id
                ).ccosy_cache.get_curvilinear_state(
                    self._world.vehicle_by_id(vehicle_id).states_cr[time_step],
                    self._world.vehicle_by_id(vehicle_id).get_lane(time_step),
                )
                is not None
            )
        ]
        
        vehicles_in_front_of_ego = [
            other_vehicle_id 
            for other_vehicle_id in valid_vehicle_ids
            if other_vehicle_id != self._ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, other_vehicle_id]
            )
        ]

        if not vehicles_in_front_of_ego:
            return True, 0.0 # No violation, as there are no vehicles in front of the ego vehicle
        
        vehicles_on_access_ramp = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._on_access_ramp_pred.evaluate_boolean(
                self._world, time_step, [vehicle_id])
        ]
            
        if not vehicles_on_access_ramp:
            return True, 0.0 # No violation, as there are no vehicles on the access ramp in front of the ego vehicle
                
        vehicles_on_main_carriageway_in_future = [
            vehicle_id
            for vehicle_id in vehicles_on_access_ramp
            if any(self._on_main_carriageway_pred.evaluate_boolean(
                self._world, t, [vehicle_id]
            )
            for t in range(time_step, min(time_step + 5, len(
                self._world.vehicle_by_id(vehicle_id).states_cr)))
                  )                 
        ]
        
        if not vehicles_on_main_carriageway_in_future:
            return True, 0.0
        
        # There is(are) vehicle(s) on the main carriageway in the future => ego must consider entering vehicles
        # Ego vehicle can be on the right lane of the main carriageway now but NOT in the future
        ego_on_main_carriageway_right_lane = self._on_main_carriageway_right_lane_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        
        ego_on_main_carriage_right_lane_in_future = any(
            self._on_main_carriageway_right_lane_pred.evaluate_boolean(
                self._world, t, [self._ego_vehicle_id])
            for t in range(time_step, min(time_step + 5, len(self._ego_vehicle.states_cr)))
        )
        
        condition = ego_on_main_carriageway_right_lane or not ego_on_main_carriage_right_lane_in_future

        if condition: 
            return True, 0.0  # No violation
        
        else: # Ego does not consider entering vehicles
            robustness = self._on_main_carriageway_right_lane_pred.evaluate_robustness(
                self._world, time_step, [self._ego_vehicle_id]
            )
            return False, robustness  
  

#-----------------------R_IN_1-Rule--------------------------------   
class StopAtStopSignChecker(RuleCheckerBase):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self._world = World(set(self._vehicles), self._road_network)
        self._stop_line_in_front_pred = PredStopLineInFront(self._config)
        self._in_stand_still_pred = PredInStandStill(self._config) 
        
    def evaluate(self, time_step: int) -> Tuple[bool, float]:
        
        stop_line_in_front = self._stop_line_in_front_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        stop_line_in_front_p = self._stop_line_in_front_pred.evaluate_boolean(
            self._world, max(time_step - 1, 0), [self._ego_vehicle_id]
        )
        
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
        
        traffic_light_red = self._traffic_light_red_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        stop_line_in_front = self._stop_line_in_front_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        ego_on_intersection = self._on_intersection_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        # ego_on_right_turn = self._on_right_turn_pred.evaluate_boolean(
        #     self._world, time_step, [self._ego_vehicle_id]
        # )
        ego_in_stand_still = self._in_stand_still_pred.evaluate_boolean(
            self._world, time_step, [self._ego_vehicle_id]
        )
        # TODO : Implementation may need to be corrected
        if traffic_light_red and stop_line_in_front and not ego_on_intersection : #and not ego_on_right_turn:
            if not ego_in_stand_still:
                # return the velocity of the ego vehicle as the cost of violation? 
                return False, self._ego_vehicle.states_cr[time_step].velocity  
        
        return True, 0.0         


 
#--------------------- Helper functions ----------------------------
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
