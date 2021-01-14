""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_id = None
intersection = network.intersections[globalParameters.intersection_id]

param sim_result = None
sim_result = globalParameters.sim_result
sim_trajectory = None

if sim_result:
	sim_trajectory = sim_result.trajectory

param blueprints = {'ego': 'vehicle.tesla.model3'}
blueprints = globalParameters.blueprints

param vehicleLightStates = None
vehicleLightStates = globalParameters.vehicleLightStates

param event_monitor = None
event_monitor = globalParameters.event_monitor

import visualization
from intersection_monitor import SignalType
import carla
from signals import vehicleLightState_from_maneuverType, signalType_from_vehicleLightState
		
#CONSTANTS
SPEED = 4
ARRIVAL_DISTANCE = 4 # meters
SPAWN_DISTANCE = 20 # meters

behavior SignalBehavior(trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)
	vehicleLightStates[self.name] = lights

behavior PassBehavior(speed, trajectory):
	do SignalBehavior(trajectory)
	while (distance from self to trajectory[2].centerline[-1]) > 5:
		do FollowTrajectoryBehavior(speed, trajectory)
	take SetBrakeAction(1)

#Ego vehicle
ego_maneuver = intersection.maneuvers[3]
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
ego = Car following roadDirection from ego_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'ego',
	with blueprint blueprints['ego'],
	with behavior PassBehavior(SPEED, ego_trajectory)

monitor egoEvents:
	signal = SignalType.from_maneuver(ego_maneuver)
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	maneuvers = intersection.maneuvers
	arrived = False
	entered = False
	exited = False
	lanes = set()
	while True:
		currentTime = simulation().currentTime
		visualization.label_car(carla_world, ego)
		inIntersection = intersection.intersects(ego)
		
		if (not arrived) and (distance from (front of ego) to intersection) < ARRIVAL_DISTANCE:
			arrived = True
			event_monitor.on_arrival(currentTime, ego, ego.lane, signal)
		if inIntersection and not entered:
			entered = True
			event_monitor.on_entrance(currentTime, ego, ego.lane)
		if entered and (not exited) and not inIntersection:
			exited = True
			event_monitor.on_exit(currentTime, ego, ego.lane)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(ego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				event_monitor.on_enterLane(currentTime, ego, lane)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				event_monitor.on_exitLane(currentTime, ego, lane)
		wait

