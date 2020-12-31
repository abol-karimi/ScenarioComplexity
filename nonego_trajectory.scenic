""" Scenario Description
Simulated a new nonego car passing through the intersection to
calculate its trajectory and its order of events.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection = network.intersections[3] # unsignalized four-way intersection in Town05
intersection = globalParameters.intersection

param sim_result = None
sim_result = globalParameters.sim_result
sim_trajectory = sim_result.trajectory
sim_actions = sim_result.actions

param blueprints = None
blueprints = globalParameters.blueprints

import intersection_monitor

import visualization
from intersection_monitor import SignalType
import carla
from signals import vehicleLightState_from_maneuverType, signalType_from_vehicleLightState

#CONSTANTS
SPEED = 5
ARRIVAL_DISTANCE = 4 # meters
SPAWN_DISTANCE = 20 + Uniform(10) # meters

behavior SignalBehavior(trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)

behavior PassBehavior(speed, trajectory):
	blueprints[self.name] = self.blueprint
	do SignalBehavior(trajectory)
	while (distance from self to trajectory[2].centerline[-1]) > 5:
		do FollowTrajectoryBehavior(speed, trajectory)
	take SetBrakeAction(1)

egoState = sim_trajectory[0]['ego']
ego = Car at egoState[0], facing egoState[1],
	with name 'ego',
	with blueprint blueprints['ego']

#PLACEMENT
nonego_maneuver = Uniform(*(intersection.maneuvers))
nonego_trajectory = [nonego_maneuver.startLane, nonego_maneuver.connectingLane, nonego_maneuver.endLane]
nonego = Car following roadDirection from nonego_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'car'+str(len(sim_trajectory[0].keys())),
	with behavior PassBehavior(SPEED, nonego_trajectory)
intersection_monitor.monitor.nonego = nonego.name

monitor nonegoEvents:
	signal = SignalType.from_maneuver(nonego_maneuver)
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	maneuvers = intersection.maneuvers
	arrived = False
	entered = False
	exited = False
	lanes = set()
	while True:
		timestamp = carla_world.get_snapshot().timestamp
		visualization.label_car(carla_world, nonego)
		inIntersection = intersection.intersects(nonego)
		
		if (not arrived) and (distance from (front of nonego) to intersection) < ARRIVAL_DISTANCE:
			arrived = True
			intersection_monitor.monitor.on_arrival(timestamp, nonego, nonego.lane, signal)
		if inIntersection and not entered:
			entered = True
			intersection_monitor.monitor.on_entrance(timestamp, nonego, nonego.lane)
		if entered and (not exited) and not inIntersection:
			exited = True
			intersection_monitor.monitor.on_exit(timestamp, nonego, nonego.lane)

		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			wasOnLane = lane in lanes
			isOnLane = lane.intersects(nonego)
			if isOnLane and not wasOnLane:
				lanes.add(lane)
				intersection_monitor.monitor.on_enterLane(timestamp, nonego, lane)
			elif wasOnLane and not isOnLane:
				lanes.remove(lane)
				intersection_monitor.monitor.on_exitLane(timestamp, nonego, lane)
		wait

