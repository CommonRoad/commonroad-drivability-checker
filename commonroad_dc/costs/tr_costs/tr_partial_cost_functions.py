# Traffic rule based partial cost functions

import numpy as np
from scipy.integrate import simps

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad_dc.costs.partial_cost_functions import PartialCostFunctionException

from .tr_evaluator import TrafficRuleEvaluator
from .utilities import CriticalScaler


  
#------------------------------------Partial--Cost-Functions---------------------------------

def r_g_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_G1: safe distance to preceding vehicle
       Here, we penalize the safe distance offset"""
    try:           
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_safe_distance()
        # extract and scale the violation robustness 
        c = [
            CriticalScaler.scale_distance(-rob_val) 
            if rob_val < 0 else 0.0
            for rob_val in rob_list
            ]
        # max cost for the trajectory
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt) 
        # final unweighted cost
        cost = simps(c, dx=scenario.dt)/critical_cost 
        return cost
    except Exception as ex:
        msg = "An exception occurred during calculation of 'safe distance' cost!"
        raise PartialCostFunctionException(msg) from ex

      

def r_g_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_G2: unnecessary braking
       Here, we penalize the braking offset from the allowed threshold"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_unnecessary_braking()
        # extract and scale the violation robustness 
        c = [
            CriticalScaler.scale_acceleration(-rob_val) 
            if rob_val < 0 else 0.0
            for rob_val in rob_list
            ]
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost  
        cost = simps(c, dx=scenario.dt)/critical_cost
        return cost  
    except Exception as ex:
        msg = "An exception occurred during calculation of 'unnecessary braking' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_g_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G3: maximum speed limit
       Here, we penalize the maximum velocity offset"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_maximum_speed_limit()
        # extract and scale the violation robustness 
        c = [
            CriticalScaler.scale_speed(-rob_val) 
            if rob_val < 0 else 0.0
            for rob_val in rob_list
            ]
        # final unweighted cost (max cost = 1); see CriticalScaler
        cost = np.max(c) 
        return cost
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'maximum speed limit' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_g_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G4: traffic flow
       Here, we penalize the offset from the allowed velocity"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_traffic_flow()
        # extract and scale the violation robustness 
        c = [
            CriticalScaler.scale_speed(-rob_val)
            if rob_val < 0 else 0.0
            for rob_val in rob_list
            ]
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost
        cost = simps(c, dx=scenario.dt)/critical_cost  
        return cost       
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'traffic flow' cost!"
        raise PartialCostFunctionException(msg) from ex
    


def r_i_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_I1: stopping
       Here, we penalize the duration of stopping"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_stopping()
        # extract the violation robustness (c) as binary 
        binary = [
                1.0 
                if rob_val < 0 else 0.0 
                for rob_val in rob_list
                ]   
        # worst case binary (all 1s)   
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt) # equivalent to (Tf-T0)*Dt
        # final unweighted cost 
        cost = simps(binary, dx=scenario.dt)/critical_cost             
        return cost  
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'no stopping' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_i_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_I2: no driving faster than left traffic
       Here, we penalize the velocity offset from the allowed offset"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_driving_faster_than_left_traffic()
        # extract and scale the violation robustness 
        c = [
            CriticalScaler.scale_speed(-rob_val) 
            if rob_val < 0 else 0.0
            for rob_val in rob_list
            ]
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost  
        cost = simps(c, dx=scenario.dt)/critical_cost     
        return cost   
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'driving faster than left traffic' cost!"
        raise PartialCostFunctionException(msg) from ex
    
   

def r_i_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I3: reversing and turning
       Here, we penalize the duration of violation"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_reversing_and_turning()
        # extract the violation robustness (c) as binary 
        binary = [
                1.0 
                if rob_val < 0 else 0.0 
                for rob_val in rob_list
                ]
        # worst case binary (all 1s)
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost
        cost = simps(binary, dx=scenario.dt)/critical_cost         
        return cost  
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'reversing and turning' cost!"
        raise PartialCostFunctionException(msg) from ex
 
                  
    
def r_i_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I4: emergency lane
       Here, we penalize the duration of violation"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_emergency_lane()
        # extract the violation robustness (c) as binary 
        binary = [
                1.0 
                if rob_val < 0 else 0.0 
                for rob_val in rob_list
                ]
        # worst case binary (all 1s)
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost
        cost = simps(binary, dx=scenario.dt)/critical_cost      
        return cost  
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'emergency lane' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_i_5_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I5: consider entering vehicles
       Here, we penalize the duration of violation"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)
        rob_list = evaluator.evaluate_consider_entering_vehicle()
        # extract the violation robustness (c) as binary 
        binary = [
                1.0 
                if rob_val < 0 else 0.0 
                for rob_val in rob_list
                ]
        # worst case binary (all 1s)
        critical_cost = simps([1.0] * len(rob_list), dx=scenario.dt)
        # final unweighted cost  
        cost = simps(binary, dx=scenario.dt)/critical_cost   
        return cost  
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'consider entering vehicles' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_in_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_1: stop at stop sign
       Here, we check if rule is violated at all along the trajectory and then assign a cost"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory)        
        rob_list = evaluator.evaluate_stop_at_stop_sign() 
        return 1.0 if any(
                         rob < 0 
                         for rob in rob_list
                         ) else 0.0
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at stop sign' cost!"
        raise PartialCostFunctionException(msg) from ex
 


def r_in_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_2: stop at red traffic light
       Here, we check if rule is violated at all along the trajectory and then assign a cost"""
    try:
        evaluator = TrafficRuleEvaluator(scenario, ego_trajectory) 
        rob_list = evaluator.evaluate_stop_at_red_light()
        return 1.0 if any(
                         rob < 0 
                         for rob in rob_list
                         ) else 0.0
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at red traffic light' cost!"
        raise PartialCostFunctionException(msg) from ex
    