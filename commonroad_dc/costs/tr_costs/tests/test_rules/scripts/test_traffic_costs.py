import unittest
from commonroad.scenario.scenario import Scenario
from commonroad.common.file_reader import CommonRoadFileReader
import os
from pathlib import Path

from commonroad_dc.costs.tr_costs.tr_partial_cost_functions import(r_g_1_cost, r_g_2_cost, r_g_3_cost, r_g_4_cost,
                                                               r_i_1_cost,  r_i_2_cost, r_i_3_cost, r_i_4_cost, r_i_5_cost,
                                                               r_in_1_cost,r_in_2_cost)




# TODO: Add apprioriate scenarios for each cost function    # DONE
# TODO: Add test cases for each cost function               # DONE
# TODO: Check if the cost function is working as expected    # DONE
# TODO: Check if the cost function is returning the expected value

# TODO: Judge vehicles differently  # DONE


class TestSafeDistanceCost(unittest.TestCase):

    def setUp(self) -> None:
        super().setUp()
        #'commonroad_dc/costs/tr_costs/tests/scenarios/DEU_test_safe_distance.xml'

        self.scenario_0 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_safe_distance.xml'
                )    
        ).open(lanelet_assignment=True)[0]
        
        self.scenario_1 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_recapture_safe_distance.xml'
                )
        ).open(lanelet_assignment=True)[0]
        
        # scenario 0
        self.trajectory0_1000 = self.scenario_0.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory0_1001 = self.scenario_0.dynamic_obstacles[1].prediction.trajectory  
        self.trajectory0_1002 = self.scenario_0.dynamic_obstacles[2].prediction.trajectory  
        self.trajectory0_1003 = self.scenario_0.dynamic_obstacles[3].prediction.trajectory  
        self.trajectory0_1004 = self.scenario_0.dynamic_obstacles[4].prediction.trajectory  
        self.trajectory0_1005 = self.scenario_0.dynamic_obstacles[5].prediction.trajectory  
        self.trajectory0_1006 = self.scenario_0.dynamic_obstacles[6].prediction.trajectory  
        self.trajectory0_1007 = self.scenario_0.dynamic_obstacles[7].prediction.trajectory  
        self.trajectory0_1008 = self.scenario_0.dynamic_obstacles[8].prediction.trajectory  
        self.trajectory0_1009 = self.scenario_0.dynamic_obstacles[9].prediction.trajectory  
        self.trajectory0_1010 = self.scenario_0.dynamic_obstacles[10].prediction.trajectory  
        
        # scenario 1
        self.trajectory1_1003 = self.scenario_1.dynamic_obstacles[3].prediction.trajectory
        self.trajectory1_1007 = self.scenario_1.dynamic_obstacles[7].prediction.trajectory
        
    def test_r_g_1_cost(self):
        # scenario 0
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_0, 
            planning_problem=None, 
            ego_trajectory=self.trajectory0_1000, 
            properties=None
            ), 
            0 , 
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,  
            planning_problem=None, 
            ego_trajectory=self.trajectory0_1001, 
            properties=None
            ),
            0,  
        )
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1002,
            properties=None
            ),
            0, 
        )
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1003,
            properties=None
            ), 
            0,
        )
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1004,
            properties=None
            ),
            0, 
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1005,
            properties=None
            ),
            0,
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1006,
            properties=None
            ),
            0,
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1007,
            properties=None
            ),
            0, 
        )
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1008,
            properties=None
            ),
            0, 
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1009,
            properties=None
            ),
            0,
        )
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1010,
            properties=None
            ),
            0, 
        )
        
       # scenario 1
        self.assertGreater(r_g_1_cost(
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1003,
            properties=None
            ),
            0, f"Vehicle 1003 did not recapture safe distance hence the cost should be > 0"
        )
        
        self.assertEqual(r_g_1_cost(
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1007,
            properties=None
            ),
            0, f"Vehicle 1007 recaptured safe distance hence the cost should be 0"
        )

 
class TestUnnecessaryBrakingCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        

        self.scenario : Scenario = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_unnecessary_braking.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory   
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory   
        self.trajectory_1005 = self.scenario.dynamic_obstacles[3].prediction.trajectory  
        self.trajectory_1006 = self.scenario.dynamic_obstacles[4].prediction.trajectory   
            
    def test_r_g_2_cost(self):
        
        self.assertEqual(r_g_2_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0
        )
        self.assertEqual(r_g_2_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1001, 
            properties=None
            ),
            0                                                               
        )
        self.assertGreater(r_g_2_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1002, 
            properties=None
            ),
            0
        )
        self.assertEqual(r_g_2_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1005, 
            properties=None
            ),
            0, f"Vehicle 1005 is not unnecessarily braking hence the cost should be 0"
        )
        self.assertEqual(r_g_2_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1006,
            properties=None
            ),  
            0, f"Vehicle 1006 is not unnecessarily braking hence the cost should be 0"
        )
        
             
class TestMaximumSpeedLimitCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_max_speed_limit.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory   
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory   
        self.trajectory_1003 = self.scenario.dynamic_obstacles[3].prediction.trajectory  

       
    def test_r_g_3_cost(self):
        
        self.assertGreater(r_g_3_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0
        )
        self.assertEqual(r_g_3_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0
        )
        self.assertGreater(r_g_3_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0
        )
        self.assertEqual(r_g_3_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1003,
            properties=None
            ),
            0
        )
        
               
class TestTrafficFlowCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_preserve_traffic_flow.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory  
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory  
        self.trajectory_1003 = self.scenario.dynamic_obstacles[3].prediction.trajectory  
        self.trajectory_1004 = self.scenario.dynamic_obstacles[4].prediction.trajectory 
        self.trajectory_1005 = self.scenario.dynamic_obstacles[5].prediction.trajectory  
                
    def test_r_g_4_cost(self):
        
        self.assertEqual(r_g_4_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0
        )
        self.assertEqual(r_g_4_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0
        )
        self.assertEqual(r_g_4_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0
        )
        self.assertGreater(r_g_4_cost( 
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1003,
            properties=None
            ),
            0, f"Vehicle 1003 is not preserving traffic flow hence the cost should be > 0"
        )
        self.assertEqual(r_g_4_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1004,
            properties=None
            ),
            0
        )
        self.assertGreater(r_g_4_cost( 
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1005,
            properties=None
            ),
            0, f"Vehicle 1005 is not preserving traffic flow hence the cost should be > 0"
        )
      
            
# TODO : Recheck the implementation of the traffic rule checker 
class TestStoppingCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_standstill.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory  
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory  
        self.trajectory_1003 = self.scenario.dynamic_obstacles[3].prediction.trajectory  
        self.trajectory_1004 = self.scenario.dynamic_obstacles[4].prediction.trajectory  
        self.trajectory_1005 = self.scenario.dynamic_obstacles[5].prediction.trajectory  
        self.trajectory_1006 = self.scenario.dynamic_obstacles[6].prediction.trajectory  
        self.trajectory_1007 = self.scenario.dynamic_obstacles[7].prediction.trajectory  
        self.trajectory_1008 = self.scenario.dynamic_obstacles[8].prediction.trajectory  
        self.trajectory_1009 = self.scenario.dynamic_obstacles[9].prediction.trajectory  
        self.trajectory_1010 = self.scenario.dynamic_obstacles[10].prediction.trajectory 
        
            
    def test_r_i_1_cost(self):
        
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0
        )
        self.assertGreater(r_i_1_cost(  
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0, f"Vehicle 1001 is stopping hence the cost should be > 0"
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1003,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1004,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1005,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1006,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1007,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1008,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1009,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1010,
            properties=None
            ),
            0
        )
  

class TestDrivingFasterThanLeftTrafficCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario_0 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_overtaking_right_normal.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.scenario_1 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_overtaking_right_congestion.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.scenario_2 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_overtaking_right_broad_lane_marking.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        # scenario 0
        self.trajectory0_1000 = self.scenario_0.dynamic_obstacles[0].prediction.trajectory  
        self.trajectory0_1001 = self.scenario_0.dynamic_obstacles[1].prediction.trajectory   
        self.trajectory0_1002 = self.scenario_0.dynamic_obstacles[2].prediction.trajectory   
        self.trajectory0_1003 = self.scenario_0.dynamic_obstacles[3].prediction.trajectory  
        
        # scenario 1
        self.trajectory1_1000 = self.scenario_1.dynamic_obstacles[0].prediction.trajectory  
        self.trajectory1_1001 = self.scenario_1.dynamic_obstacles[1].prediction.trajectory   
        
        # scenario 2
        self.trajectory2_1003 = self.scenario_2.dynamic_obstacles[3].prediction.trajectory   
                    
    def test_r_i_2_cost(self):
        
        # scenario 0 , test when in normal traffic flow
        self.assertEqual(r_i_2_cost(
            scenario=self.scenario_0,  
            planning_problem=None, 
            ego_trajectory=self.trajectory0_1000, 
            properties=None
            ), 
            0, f"Vehicle 1000 is not driving faster than left traffic hence the cost should be 0"
        )
        self.assertGreater(r_i_2_cost( 
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1001,
            properties=None
            ),
            0, f"Vehicle 1001 is driving faster than left traffic hence the cost should be > 0"
        )
        self.assertEqual(r_i_2_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1002,
            properties=None
            ),
            0
        )
        self.assertEqual(r_i_2_cost(
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1003,
            properties=None
            ),
            0
        )
        
        # scenario 1 , test when in congestion
        self.assertEqual(r_i_2_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1000,
            properties=None
            ),
            0, f"Vehicle 1000 is driving faster than left traffic but left traffic is in congestion hence the cost should be 0"
        )
        self.assertGreater(r_i_2_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1001,
            properties=None
            ),
            0
        )
        
        # scenario 2 , test when broad lane marking is present
        self.assertEqual(r_i_2_cost( 
            scenario=self.scenario_2,
            planning_problem=None,
            ego_trajectory=self.trajectory2_1003,
            properties=None
            ),
            0, f"Vehicle 1003 is driving faster than left traffic but left traffic is in congestion hence the cost should be 0"
        )


class TestReversingAndTurningCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_reversing_and_u_turn.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory  
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory   
        self.trajectory_1003 = self.scenario.dynamic_obstacles[3].prediction.trajectory  
         
    def test_r_i_3_cost(self):
        
        self.assertGreater(r_i_3_cost( 
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0, f"Vehicle 1000 is reversing hence the cost should be > 0"
        )
        self.assertGreater(r_i_3_cost( 
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0, f"Vehicle 1001 is turning hence the cost should be > 0"
        )
        self.assertGreater(r_i_3_cost( 
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0, f"Vehicle 1002 is turning hence the cost should be > 0"
        )
        self.assertEqual(r_i_3_cost( 
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1003,
            properties=None
            ),
            0, f"Vehicle 1003 is not reversing or turning hence the cost > 0"
        )
        
     
class TestEmergencyLaneCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario_0 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_emergency_three_lanes_with_shoulder.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.scenario_1 : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_emergency_two_lanes_not_broad_enough.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        # scenario_0
        self.trajectory0_1021 = self.scenario_0.dynamic_obstacles[21].prediction.trajectory 
        self.trajectory0_1016 = self.scenario_0.dynamic_obstacles[16].prediction.trajectory 
        
        # scenario_1
        self.trajectory1_1000 = self.scenario_1.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory1_1001 = self.scenario_1.dynamic_obstacles[1].prediction.trajectory 
        self.trajectory1_1006 = self.scenario_1.dynamic_obstacles[6].prediction.trajectory 
        self.trajectory1_1007 = self.scenario_1.dynamic_obstacles[7].prediction.trajectory 
        self.trajectory1_1009 = self.scenario_1.dynamic_obstacles[9].prediction.trajectory 
                      
    def test_r_i_4_cost(self):
        
        # scenario_0 : 3 lanes with shoulder
        self.assertEqual(r_i_4_cost( 
            scenario=self.scenario_0,  
            planning_problem=None, 
            ego_trajectory=self.trajectory0_1021, 
            properties=None
            ), 
            0, f"Vehicle 1021 is driving rightmost lane hence the cost should be 0"
        )
        self.assertGreater(r_i_4_cost( 
            scenario=self.scenario_0,
            planning_problem=None,
            ego_trajectory=self.trajectory0_1016,
            properties=None
            ),
            0, f"Vehicle 1016 is congestion in leftmost lane and not driving leftmost hence the cost should be > 0"
        )
        
        # scenario_1 : 2 lanes not broad enough
        self.assertGreater(r_i_4_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1000,
            properties=None
            ),
            0, f"Vehicle 1000 is driving on the emergency lane on narrow interstate hence the cost should be > 0"
        )
        self.assertEqual(r_i_4_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1001,
            properties=None
            ),
            0
        )
        self.assertGreater(r_i_4_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1006,
            properties=None
            ),
            0, f"Vehicle 1006 is driving on the emergency lane on narrow interstate hence the cost should be > 0"
        )
        self.assertGreater(r_i_4_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1007,
            properties=None
            ),
            0, f"Vehicle 1007 is driving on the emergency lane on narrow interstate hence the cost should be > 0"
        )
        self.assertGreater(r_i_4_cost( 
            scenario=self.scenario_1,
            planning_problem=None,
            ego_trajectory=self.trajectory1_1009,
            properties=None
            ),
            0, f"Vehicle 1009 is causing congestion on narrow interstate hence the cost should be > 0"
        )

     
class TestConsiderEnteringVehicleCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_consider_entering_vehicles_for_lane_change.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory
                    
    def test_r_i_5_cost(self):
        
        self.assertGreater(r_i_5_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0, f"Vehicle 1000 is not considering entering vehicle hence the cost should be > 0"
        )


        self.assertEqual(r_i_5_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0, f"Vehicle 1001 is considering entering vehicle hence the cost should be 0"
        )
        
        self.assertEqual(r_i_5_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0, f"Vehicle 1002 is considering entering vehicle hence the cost should be 0"
        )
   
class TestStopAtStopSignCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_stop_line.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_1000 = self.scenario.dynamic_obstacles[0].prediction.trajectory 
        self.trajectory_1001 = self.scenario.dynamic_obstacles[1].prediction.trajectory
        self.trajectory_1002 = self.scenario.dynamic_obstacles[2].prediction.trajectory
        self.trajectory_1003 = self.scenario.dynamic_obstacles[3].prediction.trajectory
        
    def test_r_in_1_cost(self):
        
        self.assertGreater(r_in_1_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_1000, 
            properties=None
            ), 
            0, f"Vehicle 1000 is not stopping at stop sign hence the cost should be > 0"
        )
        self.assertGreater(r_in_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1001,
            properties=None
            ),
            0, f"Vehicle 1001 is not stopping at stop sign hence the cost should be > 0"
        )
        self.assertGreater(r_in_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1002,
            properties=None
            ),
            0, f"Vehicle 1002 is stopping at stop sign hence the cost should be > 0"
        )
        self.assertEqual(r_in_1_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_1003,
            properties=None
            ),
            0, f"Vehicle 1003 is stopping at stop sign hence the cost should be 0"
        )
        

class TestStopAtTrafficLightCost(unittest.TestCase):
    
    def setUp(self) -> None:
        super().setUp()
        
        self.scenario : Scenario  = CommonRoadFileReader(
            Path(__file__).parent.parent.joinpath(
                'scenarios/DEU_test_stop_at_red_light.xml'
            )
        ).open(lanelet_assignment=True)[0]
        
        self.trajectory_31 = self.scenario.dynamic_obstacles[1].prediction.trajectory 
        self.trajectory_32 = self.scenario.dynamic_obstacles[2].prediction.trajectory
        self.trajectory_37 = self.scenario.dynamic_obstacles[7].prediction.trajectory
                    
    def test_r_in_2_cost(self):
        
        self.assertGreater(r_in_2_cost(
            scenario=self.scenario,  
            planning_problem=None, 
            ego_trajectory=self.trajectory_31, 
            properties=None
            ), 
            0, f"Vehicle 31 is not stopping at red traffic light hence the cost should be > 0"
        )
        self.assertGreater(r_in_2_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_32,
            properties=None
            ),
            0, f"Vehicle 32 is not stopping at red traffic light hence the cost should be > 0"
        )
        self.assertEqual(r_in_2_cost(
            scenario=self.scenario,
            planning_problem=None,
            ego_trajectory=self.trajectory_37,
            properties=None
            ),
            0, f"Vehicle 37 is stopping at red traffic light hence the cost should be 0"
        )
    
    

       
if __name__ == '__main__':
    unittest.main()       