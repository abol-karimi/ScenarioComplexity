""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

import visualization
import intersection_monitor
import carla

#CONSTANTS
EGO_SPEED = 3

#EGO BEHAVIOR: Follow lane
behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=EGO_SPEED)

#GEOMETRY
incomingLanes = []
# id = 2 # unsignalized T-intersection in Town05
id = 3 # unsignalized four-way intersection in Town05
# id = 11 # signalized T-intersection
intersection = network.intersections[id]
intersection_monitor.monitor.set_intersection(intersection)

#PLACEMENT
initLane = intersection.incomingLanes[0]
spawnPt = OrientedPoint on initLane.centerline
require (distance from spawnPt to initLane.maneuvers[0].intersection) > 20
require (distance from spawnPt to initLane.maneuvers[0].intersection) < 40

behavior Stop():
	while (self.speed > 1): # change to 0
		take SetBrakeAction(1)
	take SetBrakeAction(0)

behavior StopAtIntersection():
	stopped = False
	try:
		do EgoBehavior()
	interrupt when (not stopped) and (distance from (front of self) to intersection) < 4:
		intersection_monitor.monitor.on_arrival()
		do Stop()
		stopped = True
		
ego = Car at spawnPt,
	with behavior StopAtIntersection()

egoLanes = set()
monitor egoEvents:
	carla_world = simulation().world
	while True:
		elapsed_seconds = carla_world.get_snapshot().timestamp.elapsed_seconds
		for maneuver in intersection.maneuvers:
			lane = maneuver.connectingLane
			if lane.intersects(ego):
				visualization.draw_lane(carla_world, lane, color=carla.Color(255, 0, 0), life_time=0.01)
				if not lane in egoLanes:
					egoLanes.add(lane)
					intersection_monitor.monitor.events.append(intersection_monitor.EnteredLaneEvent(elapsed_seconds, ego, lane))
					print("Ego entered lane " + str(intersection_monitor.monitor.laneId[lane]) + " at " + str(elapsed_seconds) + " seconds.")
			elif lane in egoLanes:
				egoLanes.remove(lane)
				intersection_monitor.monitor.events.append(intersection_monitor.ExitedLaneEvent(elapsed_seconds, ego, lane))
				print("Ego exited lane " + str(intersection_monitor.monitor.laneId[lane]) + " at " + str(elapsed_seconds) + " seconds.")
		wait
	