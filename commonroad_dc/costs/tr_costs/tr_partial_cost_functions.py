
from typing import Dict, Any
import numpy as np

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad_dc.costs.partial_cost_functions import PartialCostFunctionException
from .tr_cost_evaluator import TrafficRuleCostEvaluator



  
#------------------------------------Partial--Cost-Functions---------------------------------

def r_g_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_G1: safe distance to preceding vehicle"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_safe_distance() 
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = "An exception occurred during calculation of 'safe distance' cost!"
        raise PartialCostFunctionException(msg) from ex

        

def r_g_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_G2: unnecessary braking"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_unnecessary_braking()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
                
        return np.max(costs)  # TODO : continuous refinement of the cost function
        
    except Exception as ex:
        msg = "An exception occurred during calculation of 'unnecessary braking' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    
def r_g_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G3: maximum speed limit"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_maximum_speed_limit()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'maximum speed limit' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_g_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G4: preserve traffic flow"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_traffic_flow()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'traffic flow' cost!"
        raise PartialCostFunctionException(msg) from ex
    


def r_i_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_I1: no stopping"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_no_stopping()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'no stopping' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_i_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_I2: no driving faster than left traffic"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
    
        output = evaluator.evaluate_driving_faster_than_left_traffic()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function

    except Exception as ex:
        msg = f"An exception occurred during calculation of 'driving faster than left traffic' cost!"
        raise PartialCostFunctionException(msg) from ex
    
   

def r_i_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I3: reversing and turning"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
               
        output = evaluator.evaluate_reversing_and_turning()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'reversing and turning' cost!"
        raise PartialCostFunctionException(msg) from ex
 
                  
    
def r_i_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I4: emergency lane"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_emergency_lane()
    
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'emergency lane' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_i_5_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I5: consider entering vehicles"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_consider_entering_vehicle()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function

    except Exception as ex:
        msg = f"An exception occurred during calculation of 'consider entering vehicles' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_in_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_1: stop at stop sign"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
                
        output = evaluator.evaluate_stop_at_stop_sign()
    
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function

    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at stop sign' cost!"
        raise PartialCostFunctionException(msg) from ex
 


def r_in_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_2: stop at red traffic light"""
    try:
        evaluator = TrafficRuleCostEvaluator(scenario, ego_trajectory)
         
        output = evaluator.evaluate_stop_at_traffic_light()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at red traffic light' cost!"
        raise PartialCostFunctionException(msg) from ex
    