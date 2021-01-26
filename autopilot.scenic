""" Scenario Description
Ego-vehicle driven by Carla's autopilot.
All nonegos' behaviors are predetermined.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_id = None
intersection = network.intersections[globalParameters.intersection_id]

param maneuver_id = None
maneuver_id = globalParameters.maneuver_id

param trajectory = None
trajectory = globalParameters.trajectory
blueprints = globalParameters.blueprints

param event_monitor = None
event_monitor = globalParameters.event_monitor

import visualization
from signals import vehicleLightState_from_maneuverType

ARRIVAL_DISTANCE = 4 # meters

behavior SignalBehavior():
	maneuvers = intersection.maneuvers
	maneuver = maneuvers[maneuver_id[self.name]]
	trajectory = [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)

behavior ReplayBehavior():
	do SignalBehavior()
	carla_world = simulation().world

	while True:
		currentTime = simulation().currentTime
		state = trajectory[currentTime][self.name]
		take SetTransformAction(state[0], state[1])
		visualization.label_car(carla_world, self)
		wait

for carName, carState in trajectory[0].items():
	if carName != 'ego':
		car = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior ReplayBehavior(),
			with physics False
	else:
		ego = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior AutopilotBehavior()

from signals import SignalType
ego_maneuver = intersection.maneuvers[maneuver_id['ego']]

monitor egoEvents:
	signal = SignalType.from_maneuver(ego_maneuver)
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
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
			event_monitor.on_arrival(currentTime, 'ego', ego.lane.uid, signal)
		if inIntersection and not entered:
			entered = True
			event_monitor.on_entrance(currentTime, 'ego', ego.lane.uid)
		if entered and (not exited) and not inIntersection:
			exited = True
			event_monitor.on_exit(currentTime, 'ego', ego.lane.uid)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(ego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				event_monitor.on_enterLane(currentTime, 'ego', lane.uid)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				event_monitor.on_exitLane(currentTime, 'ego', lane.uid)
		wait