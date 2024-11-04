
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from crmonitor.common.world import World, RoadNetwork

from .utilities import load_configuration, create_other_vehicle_from_obstacle, create_ego_vehicle_from_trajectory

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




class ScenarioInitializer:
    def __init__(self, scenario: Scenario, ego_trajectory: Trajectory, config_path: str = None):
        self._config = load_configuration(config_path)
        self._road_network = RoadNetwork(scenario.lanelet_network, self._config.get("road_network_param"))
        self._ego_vehicle = create_ego_vehicle_from_trajectory(
            ego_trajectory, self._road_network,  self._config
        )
        self._other_vehicles = [ create_other_vehicle_from_obstacle(
                                obstacle, self._road_network, self._config
                                )
                                for obstacle in scenario.dynamic_obstacles
        ]
        self._vehicles = [self._ego_vehicle] + self._other_vehicles
        self._world = World(set(self._vehicles), self._road_network, scenario)
        self._init_predicates()


    @property
    def world(self):
        return self._world

    @property
    def ego_vehicle_id(self):
        return self._ego_vehicle.id
    

    def _init_predicates(self):        
        self._in_same_lane_pred = PredInSameLane(self._config)
        self._in_front_of_pred = PredInFrontOf(self._config)
        self._safe_distance_pred = PredSafeDistPrec(self._config)
        self._cut_in_pred = PredCutIn(self._config)
        self._abrupt_braking_pred = PredAbruptBreaking(self._config)
        self._rel_abrupt_braking_pred = PredRelAbruptBreaking(self._config)
        self._slow_leading_vehicle_pred = PredSlowLeadingVehicle(self._config)
        self._preserves_traffic_flow_pred = PredPreservesTrafficFlow(self._config)
        self._in_stand_still_pred = PredInStandStill(self._config)
        self._standing_leading_vehicle_pred = PredExistStandingLeadingVehicle(self._config)
        self._in_congestion_pred = PredInCongestion(self._config)
        self._left_of_pred = PredLeftOf(self._config)
        self._drives_faster_pred = PredDrivesFaster(self._config)
        self._in_slow_moving_traffic_pred = PredInSlowMovingTraffic(self._config)
        self._in_queue_pred = PredInQueueOfVehicles(self._config)
        self._drives_slightly_higher_speed_pred = PredDrivesWithSlightlyHigherSpeed(self._config)
        self._right_of_broad_lane_marking_pred = PredRightOfBroadLaneMarking(self._config)
        self._left_of_broad_lane_marking_pred = PredLeftOfBroadLaneMarking(self._config)
        self._on_access_ramp_pred = PredOnAccessRamp(self._config)
        self._on_main_carriageway_pred = PredOnMainCarriageway(self._config)
        
