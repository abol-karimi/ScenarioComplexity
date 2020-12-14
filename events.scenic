""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

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

forkId = {}
forks = intersection.incomingLanes
for index in range(len(forks)):
	forkId[forks[index]] = index

exitId = {}
exits = intersection.outgoingLanes
for index in range(len(exits)):
	exitId[exits[index]] = index

maneuverId = {}
laneId = {}
maneuvers = intersection.maneuvers
for index in range(len(maneuvers)):
	maneuver = maneuvers[index]
	maneuverId[maneuver] = index
	laneId[maneuver.connectingLane] = index
	fork = maneuver.startLane
	exit = maneuver.endLane
	print("Lane from " + "F" + str(forkId[fork]) + " to E" + str(exitId[exit]) + " is " + maneuver.type.name)

for maneuver in maneuvers:
	for conflict in maneuver.conflictingManeuvers:
		print("overlaps(" + str(maneuverId[maneuver]) + ", " + str(maneuverId[conflict]) + ")")

initLane = intersection.incomingLanes[0]

#PLACEMENT
spawnPt = OrientedPoint on initLane.centerline
require (distance from spawnPt to initLane.maneuvers[0].intersection) > 20
require (distance from spawnPt to initLane.maneuvers[0].intersection) < 40

behavior Stop():
	while (self.speed > 1): # change to 0
		take SetBrakeAction(1)
	take SetBrakeAction(0)

def draw_lane(lane, color=carla.Color(255,0,0), life_time=-1):
	carla_world = simulation().world
	locations = [carla.Location(p[0], -p[1], 0.1) for p in lane.leftEdge.lineString.coords]
	for i in range(len(locations)-1):
		begin = locations[i]
		end = locations[i+1]
		carla_world.debug.draw_line(begin, end, thickness=0.1, color=color, life_time=life_time)
	locations = [carla.Location(p[0], -p[1], 0.1) for p in lane.rightEdge.lineString.coords]
	for i in range(len(locations)-1):
		begin = locations[i]
		end = locations[i+1]
		carla_world.debug.draw_line(begin, end, thickness=0.1, color=color, life_time=life_time)	

behavior StopAtIntersection():
	stopped = False
	try:
		do EgoBehavior()
	interrupt when (not stopped) and (distance from (front of self) to intersection) < 4:
		do Stop()
		stopped = True
		
ego = Car at spawnPt,
	with behavior StopAtIntersection()

egoLanes = set()
monitor egoEvents:
	carla_world = simulation().world
	while True:
		elapsed_seconds = carla_world.get_snapshot().timestamp.elapsed_seconds
		for maneuver in maneuvers:
			lane = maneuver.connectingLane
			if lane.intersects(ego):
				draw_lane(lane, color=carla.Color(255, 0, 0), life_time=0.01)
				if not lane in egoLanes:
					egoLanes.add(lane)
					print("Ego entered lane " + str(laneId[lane]) + " at " + str(elapsed_seconds) + " seconds.")
			elif lane in egoLanes:
				egoLanes.remove(lane)
				print("Ego exited lane " + str(laneId[lane]) + " at " + str(elapsed_seconds) + " seconds.")
		wait
	