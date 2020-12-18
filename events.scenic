""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

import visualization
import intersection_monitor
import carla
from signals import vehicleLightState_from_maneuverType

#CONSTANTS
EGO_SPEED = 3
ARRIVAL_DISTANCE = 4 # meters
SPAWN_DISTANCE = 20 # meters

behavior Stop():
	while (self.speed > 0):
		take SetBrakeAction(1)
	take SetBrakeAction(0)

#EGO BEHAVIOR: stop at intersection and go
behavior EgoBehavior(target_speed, trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)
	carla_world = simulation().world
	stopped = False
	try:
		do FollowTrajectoryBehavior(target_speed, trajectory)
	interrupt when (not stopped) and (distance from (front of self) to trajectory[1]) < ARRIVAL_DISTANCE:
		timestamp = carla_world.get_snapshot().timestamp.frame
		intersection_monitor.monitor.on_arrival(timestamp, self, self.lane)
		do Stop()
		stopped = True

#GEOMETRY
incomingLanes = []
# id = 2 # unsignalized T-intersection in Town05
id = 3 # unsignalized four-way intersection in Town05
# id = 11 # signalized T-intersection
intersection = network.intersections[id]
intersection_monitor.monitor.set_intersection(intersection)

#PLACEMENT
maneuvers = intersection.maneuvers
maneuver = Uniform(*maneuvers)
trajectory = [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]
spawnPt = maneuver.startLane.flowFrom(maneuver.startLane.centerline[-1], -15)
	
ego = Car following roadDirection from maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'ego',
	with behavior EgoBehavior(EGO_SPEED, trajectory)

monitor egoEvents:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	entered = False
	exited = False
	egoLanes = set()
	while True:
		for maneuver in intersection.maneuvers:
			lane = maneuver.connectingLane
			if lane.intersects(ego):
				visualization.draw_lane(carla_world, lane, color=carla.Color(255, 0, 0), life_time=0.1)
				if not lane in egoLanes:
					egoLanes.add(lane)
					timestamp = carla_world.get_snapshot().timestamp.frame
					intersection_monitor.monitor.on_enterLane(timestamp, ego, lane)
			elif lane in egoLanes:
				egoLanes.remove(lane)
				timestamp = carla_world.get_snapshot().timestamp.frame
				intersection_monitor.monitor.on_exitLane(timestamp, ego, lane)
			if (not entered) and intersection.intersects(ego):
				entered = True
				timestamp = carla_world.get_snapshot().timestamp.frame
				intersection_monitor.monitor.on_entrance(timestamp, ego, ego.lane)
			if entered and (not exited) and not intersection.intersects(ego):
				exited = True
				timestamp = carla_world.get_snapshot().timestamp.frame
				intersection_monitor.monitor.on_exit(timestamp, ego, ego.lane)
		wait
	