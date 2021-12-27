""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_uid = None
intersection = network.elements[globalParameters.intersection_uid]

param car_name = None
car_name = globalParameters.car_name

param maneuver_uid = None
maneuver_uid = globalParameters.maneuver_uid

param car_blueprint = None
car_blueprint = globalParameters.car_blueprint

param car_size = None
car_size = globalParameters.car_size

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

behavior PassBehavior(speed, trajectory, maneuverType):
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)
	try:
		do FollowTrajectoryBehavior(speed, trajectory)
	interrupt when (distance from self to trajectory[2].centerline[-1]) <= 5:
		abort
	do FollowLaneBehavior(target_speed=1)

#Ego vehicle
l0_uid, l1_uid, l2_uid = maneuver_uid
l0 = network.elements[l0_uid]
l1 = network.elements[l1_uid]
l2 = network.elements[l2_uid]
maneuverType = ManeuverType.guessTypeFromLanes(l0, l2, l1)
ego = Car following roadDirection from l0.centerline[-1] for -spawn_distance,
	with name car_name,
	with blueprint car_blueprint,
	with behavior PassBehavior(SPEED, [l0, l1, l2], maneuverType)


signal = SignalType.from_maneuverType(maneuverType)

monitor ego_events:
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
			event_monitor.on_arrival(ego.name, ego.lane.uid, signal.name.lower(), currentTime)
		if inIntersection and not entered:
			entered = True
			event_monitor.on_entrance(ego.name, ego.lane.uid, currentTime)
		if entered and (not exited) and not inIntersection:
			exited = True
			event_monitor.on_exit(ego.name, ego.lane.uid, currentTime)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(ego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				event_monitor.on_enterLane(ego.name, lane.uid, currentTime)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				event_monitor.on_exitLane(ego.name, lane.uid, currentTime)
		wait

monitor record_properties:
	car_size['width'] = ego.width
	car_size['length'] = ego.length
	wait