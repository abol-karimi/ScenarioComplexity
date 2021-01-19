""" Scenario Description
Simulated a new nonego car passing through the intersection to
calculate its trajectory and its order of events.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_id = None
intersection = network.intersections[globalParameters.intersection_id]

param maneuver_id = None
maneuver_id = globalParameters.maneuver_id

param sim_result = None

param blueprints = None
blueprints = globalParameters.blueprints

param vehicleLightStates = None
vehicleLightStates = globalParameters.vehicleLightStates

param event_monitor = None
event_monitor = globalParameters.event_monitor

param carName = None
carName = globalParameters.carName

import visualization
import carla
from signals import vehicleLightState_from_maneuverType, signalType_from_vehicleLightState, SignalType

#CONSTANTS
SPEED = 4
ARRIVAL_DISTANCE = 4 # meters
SPAWN_DISTANCE = 20 + Uniform(10) # meters

behavior SignalBehavior(trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)
	vehicleLightStates[self.name] = lights

behavior PassBehavior(speed, trajectory):
	blueprints[self.name] = self.blueprint
	do SignalBehavior(trajectory)
	while (distance from self to trajectory[2].centerline[-1]) > 5:
		do FollowTrajectoryBehavior(speed, trajectory)
	take SetBrakeAction(1)

#PLACEMENT
maneuvers = intersection.maneuvers
conflicts = maneuvers[maneuver_id['ego']].conflictingManeuvers
conflicts_ids = [i for i in range(len(maneuvers)) if maneuvers[i] in conflicts]
new_ids = [i for i in conflicts_ids if not i in maneuver_id.values()]
nonego_maneuver_id = new_ids[0]
maneuver_id[carName] = nonego_maneuver_id
nonego_maneuver = maneuvers[nonego_maneuver_id]
nonego_trajectory = [nonego_maneuver.startLane, nonego_maneuver.connectingLane, nonego_maneuver.endLane]
nonego = Car following roadDirection from nonego_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name carName,
	with behavior PassBehavior(SPEED, nonego_trajectory)

ego = nonego

monitor nonegoEvents:
	signal = SignalType.from_maneuver(nonego_maneuver)
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	arrived = False
	entered = False
	exited = False
	lanes = set()
	while True:
		currentTime = simulation().currentTime
		visualization.label_car(carla_world, nonego)
		inIntersection = intersection.intersects(nonego)
		
		if (not arrived) and (distance from (front of nonego) to intersection) < ARRIVAL_DISTANCE:
			arrived = True
			event_monitor.on_arrival(currentTime, nonego, nonego.lane, signal)
		if inIntersection and not entered:
			entered = True
			event_monitor.on_entrance(currentTime, nonego, nonego.lane)
		if entered and (not exited) and not inIntersection:
			exited = True
			event_monitor.on_exit(currentTime, nonego, nonego.lane)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(nonego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				event_monitor.on_enterLane(currentTime, nonego, lane)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				event_monitor.on_exitLane(currentTime, nonego, lane)
		wait

