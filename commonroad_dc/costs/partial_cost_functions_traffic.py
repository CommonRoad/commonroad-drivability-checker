from typing import Dict, Any
import numpy as np

from commonroad.planning.planning_problem import PlanningProblem

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad_dc.costs.partial_cost_functions import PartialCostFunctionException
from commonroad_dc.costs.traffic_rule_checker import (SafeDistanceChecker, UnnecessaryBrakingChecker, MaximumSpeedLimitChecker, 
                                    TrafficFlowChecker, NoStoppingChecker, DrivingFasterThanLeftTrafficChecker,
                                    ReversingAndTurningChecker, EmergencyLaneChecker, ConsiderEnteringVehicleChecker,
                                    StopAtStopSignChecker, StopAtRedLightChecker)
    

#------------------------------------Partial--Cost-Functions---------------------------------

def r_g_1_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
                
    """R_G1: safe distance to preceding vehicle"""
    try:
        checker = SafeDistanceChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'safe distance' cost!"
        raise PartialCostFunctionException(msg) from ex
        

   
def r_g_2_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
               
    """R_G2: unnecessary braking"""
    try:
        checker = UnnecessaryBrakingChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'unnecessary braking' cost!"
        raise PartialCostFunctionException(msg) from ex

    
    
def r_g_3_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
                
    """R_G3: maximum speed limit"""
    try:
        checker = MaximumSpeedLimitChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'maximum speed limit' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_g_4_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
                
    """R_G4: preserve traffic flow"""
    try:
        checker = TrafficFlowChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'traffic flow' cost!"
        raise PartialCostFunctionException(msg) from ex
    


def r_i_1_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
                
    """R_I1: no stopping"""
    try:
        checker = NoStoppingChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'no stopping' cost!"
        raise PartialCostFunctionException(msg) from ex
    
    

def r_i_2_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:
    
    """R_I2: no driving faster than left traffic"""
    try:
        checker = DrivingFasterThanLeftTrafficChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'driving faster than left traffic' cost!"
        raise PartialCostFunctionException(msg) from ex
    
   

def r_i_3_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:

    """R_I3: reversing and turning"""
    try:
        checker = ReversingAndTurningChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'reversing and turning' cost!"
        raise PartialCostFunctionException(msg) from ex
 
                  
    
def r_i_4_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:

    """R_I4: emergency lane"""
    try:
        checker = EmergencyLaneChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'emergency lane' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_i_5_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:

    """R_I5: consider entering vehicles"""
    try:
        checker = ConsiderEnteringVehicleChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'consider entering vehicles' cost!"
        raise PartialCostFunctionException(msg) from ex



def r_in_1_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:

    """R_IN_1: stop at stop sign"""
    try:
        checker = StopAtStopSignChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) # TODO refine cost value
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at stop sign' cost!"
        raise PartialCostFunctionException(msg) from ex
 


def r_in_2_cost(scenario: Scenario, planning_problem: PlanningProblem, ego_trajectory: Trajectory, properties) -> float:

    """R_IN_2: stop at red traffic light"""
    try:
        checker = StopAtRedLightChecker(scenario, ego_trajectory)
        
        time_steps = [state.time_step for state in ego_trajectory.state_list]
        
        robustness = [checker.evaluate(time_step)[1] for time_step in time_steps]
        
        return np.sum(robustness) 
    
    except Exception as ex:
        msg = f"An exception occurred during calculation of 'stop at red traffic light' cost!"
        raise PartialCostFunctionException(msg) from ex