from typing import Dict, Any

from commonroad.planning.planning_problem import PlanningProblem

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad_dc.costs.route_matcher import SolutionProperties
from commonroad_dc.costs.partial_cost_functions import PartialCostFunctionException
from commonroad_dc.costs.traffic_rule_checker import TrafficRuleChecker
    

 

def r_g_1_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory,
               properties: Dict[SolutionProperties, Dict[int, Any]]) -> float:
    
    """safe distance to preceeding vehicle """    
    try:
        # Initialize the checker
        traffic_rule_checker = TrafficRuleChecker(scenario, ego_trajectory, planning_problem)
        
        total_violation_cost = 0.0
        
        # Loop through each state in the ego vehicle's trajectory
        for state in ego_trajectory.state_list:
            time_step = state.time_step
            
            # Check if safe distance is maintained
            safe_distance_maintained, _ , actual_distance, safe_distance = traffic_rule_checker.check_safe_distance(time_step)
            
            if not safe_distance_maintained:
                # Calculate the robustness (violation)
                # print(f"Distance = {actual_distance}, safe_distance_threshold = {safe_distance}")
                violation = (safe_distance - actual_distance) * scenario.dt
                total_violation_cost += violation
                
        return total_violation_cost
    
    except Exception as ex:
        msg = "An exception occurred during calculation of 'safe distance' cost!"
        raise PartialCostFunctionException(msg) from ex

        
def r_g_2_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_G2: unnecessary braking"""   
    try:
        traffic_rule_checker = TrafficRuleChecker(scenario, ego_trajectory, planning_problem)
        
        total_violation_cost = 0.0
        
        for state in ego_trajectory.state_list:
            time_step = state.time_step
            
            no_unnecessary_braking, violation = traffic_rule_checker.check_no_unnecessary_braking(time_step)
            
            if not no_unnecessary_braking: # braked unnecessarily
                total_violation_cost += violation
        
        return total_violation_cost
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'unnecessary breaking' cost!"
        raise PartialCostFunctionException(msg) from ex
   
    
def r_g_3_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_G3:maximum speed limit"""
    try:
        pass # TODO
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'maximum speed limit' cost!"
        raise PartialCostFunctionException(msg) from ex  
    
    
def r_g_4_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_G4: traffic flow"""
    try:
        pass # TODO
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'traffic flow' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    
def r_i_1_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_I1: stopping"""
    try:
        pass # TODO
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stopping' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    
    
def r_i_2_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_I2: driving faster than left traffic"""
    try:
        pass # TODO
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'driving faster than left traffic' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    
    
def r_i_3_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_I3: reversing and turning"""
    try:
        pass # TODO
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'reversing and turning' cost!"
        raise PartialCostFunctionException(msg) from ex
 
        
                
def r_i_4_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_I4: emergency lane"""
    try:
        pass # TODO
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'emergency lane' cost!"
        raise PartialCostFunctionException(msg) from ex
    

def r_i_5_cost(scenario: Scenario, planning_problem: PlanningProblem, trajectory: Trajectory,
                properties: Dict[SolutionProperties, Dict[int,Any]]) -> float:
    """R_I4: consider entering vehicles"""
    try:
        pass # TODO
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'consider entering vehicles' cost!"
        raise PartialCostFunctionException(msg) from ex
    
