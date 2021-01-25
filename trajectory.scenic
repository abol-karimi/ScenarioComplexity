""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_id = None
intersection = network.intersections[globalParameters.intersection_id]

param car_name = None
car_name = globalParameters.car_name

param maneuver_id = None
maneuver_id = globalParameters.maneuver_id

param car_blueprint = None
car_blueprint = globalParameters.car_blueprint

param event_monitor = None
event_monitor = globalParameters.event_monitor

param spawn_distance = None
spawn_distance = globalParameters.spawn_distance

import visualization
import carla
from signals import vehicleLightState_from_maneuverType, signalType_from_vehicleLightState, SignalType
		
#CONSTANTS
SPEED = 4
ARRIVAL_DISTANCE = 4 # meters

behavior SignalBehavior(trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)

behavior PassBehavior(speed, trajectory):
	do SignalBehavior(trajectory)
	while (distance from self to trajectory[2].centerline[-1]) > 5:
		do FollowTrajectoryBehavior(speed, trajectory)
	take SetBrakeAction(1)

#Ego vehicle
car_maneuver = intersection.maneuvers[maneuver_id]
car_trajectory = [car_maneuver.startLane, car_maneuver.connectingLane, car_maneuver.endLane]
ego = Car following roadDirection from car_maneuver.startLane.centerline[-1] for -spawn_distance,
	with name car_name,
	with blueprint car_blueprint,
	with behavior PassBehavior(SPEED, car_trajectory)

monitor trafficRules_events:
	signal = SignalType.from_maneuver(car_maneuver)
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
			event_monitor.on_arrival(currentTime, ego.name, ego.lane.uid, signal)
		if inIntersection and not entered:
			entered = True
			event_monitor.on_entrance(currentTime, ego.name, ego.lane.uid)
		if entered and (not exited) and not inIntersection:
			exited = True
			event_monitor.on_exit(currentTime, ego.name, ego.lane.uid)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(ego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				event_monitor.on_enterLane(currentTime, ego.name, lane.uid)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				event_monitor.on_exitLane(currentTime, ego.name, lane.uid)
		wait

