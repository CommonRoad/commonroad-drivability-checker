# This file contains the implementation of the traffic rule checker for the commonroad scenario
from typing import Tuple
from commonroad_dc.costs.tr_costs.scenario_initializer import ScenarioInitializer
from crmonitor.evaluation.evaluation import RuleEvaluator




class SafeDistanceEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G1"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()

    # back-up evaluation method incases of failure of the RuleEvaluator
    def evaluate_step(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            v.id
            for v in self._world.vehicles
            if v.is_valid(time_step)
            and v.rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None
            and v.get_lane(time_step) is not None
            and v.ccosy_cache.get_curvilinear_state(
                v.states_cr[time_step], v.get_lane(time_step)
            ) is not None 
        ]
        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self.ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
        ] 
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
        ]
        if not vehicles_in_front_of_and_in_same_lane_as_ego: # No vehicles in front of ego in the same lane
            return 0.0

        sorted_vehicles = sorted(
            vehicles_in_front_of_and_in_same_lane_as_ego,
            key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
        )
        preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front
        safe_distance_kept = self._safe_distance_pred.evaluate_boolean(
            self._world, time_step, [self.ego_vehicle_id, preceding_vehicle_id]
        )                                              
        if safe_distance_kept: 
            return 0.0  
        else: # Safe distance is violated 
            t_c = 3  
            preceding_vehicle = self._world.vehicle_by_id(preceding_vehicle_id)
            previous_start_time = max(0, time_step - 3)
            valid_time_steps = range(previous_start_time, time_step)
            
            cut_in_happened = any(self._cut_in_pred.evaluate_boolean(
                    self._world, t, [preceding_vehicle_id, self.ego_vehicle_id]
                    )
                    for t in valid_time_steps
                    if (
                        self._ego_vehicle.is_valid(t) and preceding_vehicle.is_valid(t)
                        and preceding_vehicle.get_lane(t) is not None
                        and preceding_vehicle.ccosy_cache.get_curvilinear_state(
                            preceding_vehicle.states_cr[t],
                            preceding_vehicle.get_lane(t)
                        ) is not None
                    )
            )
            ego_recaptured_safe_distance = self._safe_distance_pred.evaluate_boolean(
                self._world, time_step + t_c, [self.ego_vehicle_id, preceding_vehicle_id]
            ) if (preceding_vehicle.is_valid(time_step + t_c) 
                  and self._ego_vehicle.is_valid(time_step + t_c)
                ) else True
            
            if cut_in_happened and ego_recaptured_safe_distance:
                return 0.0
            robustness = self._safe_distance_pred.evaluate_robustness(
                self._world, time_step, [self.ego_vehicle_id, preceding_vehicle_id]
            ) # returns (actual_distance - safe_distance) => -ve value for non-compliance
            return robustness    
 
 
class UnnecessaryBrakingEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G2"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()

    def evaluate_step(self, time_step: int) -> Tuple[bool, float]:
        valid_vehicle_ids = [
            v.id
            for v in self._world.vehicles
            if v.is_valid(time_step)
            and v.rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None
            and v.get_lane(time_step) is not None
            and v.ccosy_cache.get_curvilinear_state(
                v.states_cr[time_step], v.get_lane(time_step)
            ) is not None 
        ]
        vehicles_in_front_of_ego = [
            vehicle_id
            for vehicle_id in valid_vehicle_ids
            if vehicle_id != self.ego_vehicle_id
            and self._in_front_of_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
        ]
        vehicles_in_front_of_and_in_same_lane_as_ego = [
            vehicle_id 
            for vehicle_id in vehicles_in_front_of_ego
            if self._in_same_lane_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
        ]
        if not vehicles_in_front_of_and_in_same_lane_as_ego: #  Case 1 No preceding vehicle, check only abrupt braking
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id]
            )
            
            if not braked_abruptly:
                return 0.0   
               
            else:
                robustness = self._abrupt_braking_pred.evaluate_robustness(
                    self._world, time_step, [self.ego_vehicle_id]
                )
                
                return robustness     
    
        else: # Case 2 : There is a preceding vehicle, check both abrupt braking and relative abrupt braking
            sorted_vehicles = sorted(
                vehicles_in_front_of_and_in_same_lane_as_ego,
                key=lambda vehicle_id: self._in_front_of_pred.evaluate_robustness(
                    self._world, time_step, [self.ego_vehicle_id, vehicle_id]
                )
            )
            preceding_vehicle_id = sorted_vehicles[0]  # Get the closest vehicle in front         
            # Using robustness as the cost of violation
            braked_abruptly = self._abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id]
            )
            
            braked_abruptly_relative = self._rel_abrupt_braking_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, preceding_vehicle_id]
            )
            
            safe_distance_kept = self._safe_distance_pred.evaluate_boolean(
                self._world, time_step, [self.ego_vehicle_id, preceding_vehicle_id]
            )
            # Using CNF resolution of the temporal logic 
            condition = (not braked_abruptly
                         or not safe_distance_kept
                         or not braked_abruptly_relative) 
 
            if condition: # True => abrupt braking
                robustness = self._rel_abrupt_braking_pred.evaluate_robustness(
                    self._world, time_step, [self.ego_vehicle_id, preceding_vehicle_id]
                )
                return robustness
                       
            else:
                # No relative abrupt braking
                return 0.0
            
class MaximumSpeedLimitEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G3"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()


class TrafficFlowEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_G4"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    

class NoStoppingEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I1"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    
    
class DrivingFasterThanLeftTrafficEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I2"
        )
        
    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    
    # back-up evaluation method
    def evaluate_step(self, time_step) -> float:
        valid_vehicle_ids = [
            v.id
            for v in self._world.vehicles
            if v.is_valid(time_step)
            and v.rear_s(time_step, self._ego_vehicle.get_lane(time_step)) is not None
            and v.get_lane(time_step) is not None
            and v.ccosy_cache.get_curvilinear_state(
                v.states_cr[time_step], v.get_lane(time_step)
            ) is not None 
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
            return 0.0

        vehicles_ego_drives_faster_than = [
            vehicle_id
            for vehicle_id in vehicles_on_the_left_of_ego
            if self._drives_faster_pred.evaluate_boolean(
                self._world, time_step, [self._ego_vehicle_id, vehicle_id]
            )
        ]
        if not vehicles_ego_drives_faster_than:  # ego is not driving faster than any vehicle on its left
            return 0.0  # No violation
        
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
                return 0.0  # No violation
            
            robustness = self._drives_faster_pred.evaluate_robustness(
                self._world, time_step, [self.ego_vehicle_id, vehicle_id]
            )
            
            return False, robustness  # Violation occurred


class ReversingAndTurningEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I3"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    

class EmergencyLaneEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I4"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    
    
class ConsiderEnteringVehicleEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_I5"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    
   
class StopAtStopSignEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN1_past"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
    
    
class StopAtRedLightEvaluator(ScenarioInitializer):
    def __init__(self, scenario, ego_trajectory):
        super().__init__(scenario, ego_trajectory)
        self.rule_evaluator = RuleEvaluator.create_from_config(
            world=self.world, 
            ego_id=self.ego_vehicle_id, 
            rule="R_IN2_TOR"
        )

    def evaluate_full(self) -> list[float]:
        return self.rule_evaluator.evaluate()
