from typing import Dict, Any
import numpy as np
import logging

from commonroad.planning.planning_problem import PlanningProblem

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad_dc.costs.partial_cost_functions import PartialCostFunctionException
from .traffic_rule_evaluators import *

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TR_COSTFUNCTION")

    

#------------------------------------Partial--Cost-Functions---------------------------------

def r_g_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    """R_G1: safe distance to preceding vehicle"""
    
    try:
        evaluator = SafeDistanceEvaluator(scenario, ego_trajectory)
        
        try:
            output = evaluator.evaluate_full()
            logger.info("Evaluating R_G1 using RuleEvaluator")
            costs = [-x if x < 0 else 0.0 for x in output ] 
            return np.sum(costs) # TODO : continuous refinement of the cost function
        
        except Exception:
            logger.info("Evaluating R_G1 Manually")
            time_steps = [state.time_step for state in ego_trajectory.state_list]
            output = [evaluator.evaluate_step(time_step) for time_step in time_steps]
            costs = [-x if x < 0 else 0.0 for x in output ] 
            return np.sum(costs)
    
    except Exception as ex:
        msg = "An exception occurred during calculation of 'safe distance' cost!"
        raise PartialCostFunctionException(msg) from ex

        

def r_g_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    """R_G2: unnecessary braking"""
    
    try:
        evaluator = UnnecessaryBrakingEvaluator(scenario, ego_trajectory)
        
        try:
            output = evaluator.evaluate_full()
            logger.info("Evaluating R_G2 using RueEvaluator")
            costs = [-x if x < 0 else 0.0 for x in output ] 
            return np.sum(costs)  # TODO : continuous refinement of the cost function
        
        except Exception:
            logger.info("Evaluating R_G2 Manually")
            time_steps = [state.time_step for state in ego_trajectory.state_list]
            output = [evaluator.evaluate_step(time_step) for time_step in time_steps]
            costs = [-x if x < 0 else 0.0 for x in output ] 
            return np.sum(costs)
    
    except Exception as ex:
        msg = "An exception occurred during calculation of 'unnecessary braking' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    
def r_g_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G3: maximum speed limit"""
    try:
        evaluator = MaximumSpeedLimitEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'maximum speed limit' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_g_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_G4: preserve traffic flow"""
    try:
        evaluator = TrafficFlowEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'traffic flow' cost!"
        raise PartialCostFunctionException(msg) from ex
    


def r_i_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
                
    """R_I1: no stopping"""
    try:
        evaluator = NoStoppingEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs) # TODO : continuous refinement of the cost function
        
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'no stopping' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_i_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:
    
    """R_I2: no driving faster than left traffic"""
    try:
        evaluator = DrivingFasterThanLeftTrafficEvaluator(scenario, ego_trajectory)

        try:
            output = evaluator.evaluate_full()
            logger.info("Evaluating R_I2 using RuleEvaluator")
            costs = [-x if x < 0 else 0.0 for x in output ] 
            
            return np.sum(costs) # TODO : continuous refinement of the cost function
        
        except Exception:
            
            logger.info("Evaluating R_I2 Manually")
            time_steps = [state.time_step for state in ego_trajectory.state_list]
            output = [evaluator.evaluate_step(time_step) for time_step in time_steps]
            costs = [-x if x < 0 else 0.0 for x in output ] 
            return np.sum(costs)
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'driving faster than left traffic' cost!"
        raise PartialCostFunctionException(msg) from ex
    
   

def r_i_3_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I3: reversing and turning"""
    try:
        evaluator = ReversingAndTurningEvaluator(scenario, ego_trajectory)
               
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs)
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'reversing and turning' cost!"
        raise PartialCostFunctionException(msg) from ex
 
                  
    
def r_i_4_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I4: emergency lane"""
    try:
        evaluator = EmergencyLaneEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_full()
    
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs)
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'emergency lane' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_i_5_cost(scenario: Scenario, planning_problem: PlanningProblem,
               ego_trajectory: Trajectory, properties) -> float:

    """R_I5: consider entering vehicles"""
    try:
        evaluator = ConsiderEnteringVehicleEvaluator(scenario, ego_trajectory)
        
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs)

    except Exception as ex:
        msg = f"An exception occurred during calculation of 'consider entering vehicles' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_in_1_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_1: stop at stop sign"""
    try:
        evaluator = StopAtStopSignEvaluator(scenario, ego_trajectory)
                
        output = evaluator.evaluate_full()
    
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs)

    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at stop sign' cost!"
        raise PartialCostFunctionException(msg) from ex
 


def r_in_2_cost(scenario: Scenario, planning_problem: PlanningProblem,
                ego_trajectory: Trajectory, properties) -> float:

    """R_IN_2: stop at red traffic light"""
    try:
        evaluator = StopAtRedLightEvaluator(scenario, ego_trajectory)
         
        output = evaluator.evaluate_full()
        
        costs = [-x if x < 0 else 0.0 for x in output ] 
        
        return np.sum(costs)
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at red traffic light' cost!"
        raise PartialCostFunctionException(msg) from ex
    