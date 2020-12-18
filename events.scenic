""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

import visualization
import intersection_monitor
from intersection_monitor import CarState, SignalType
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
ego_maneuver = Uniform(*(intersection.maneuvers))
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
car1_maneuver = Uniform(*(ego_maneuver.conflictingManeuvers))
car1_trajectory = [car1_maneuver.startLane, car1_maneuver.connectingLane, car1_maneuver.endLane]

ego = Car following roadDirection from ego_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'ego',
	with behavior EgoBehavior(EGO_SPEED, ego_trajectory)

car1 = Car following roadDirection from car1_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'car1',
	with behavior FollowTrajectoryBehavior(EGO_SPEED, car1_trajectory)

cars = [ego, car1]

monitor carEvents:
	turnSignal = {ego.name:SignalType.from_maneuver(ego_maneuver), 
							car1.name:SignalType.from_maneuver(car1_maneuver)}
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	maneuvers = intersection.maneuvers
	carState = {car:CarState() for car in cars}
	while True:
		timestamp = carla_world.get_snapshot().timestamp.frame
		for car in cars:
			visualization.label_car(carla_world, car)
			state = carState[car]
			arrived = state.arrived
			entered = state.entered
			exited = state.exited
			inIntersection = intersection.intersects(car)
			signal = turnSignal[car.name]
			
			if (not arrived) and (distance from (front of car) to intersection) < ARRIVAL_DISTANCE:
				carState[car].arrived = True
				intersection_monitor.monitor.on_arrival(timestamp, car, car.lane, signal)
			if inIntersection and not entered:
				carState[car].entered = True
				intersection_monitor.monitor.on_entrance(timestamp, car, car.lane)
			if entered and (not exited) and not inIntersection:
				carState[car].exited = True
				intersection_monitor.monitor.on_exit(timestamp, car, car.lane)

			for maneuver in maneuvers:
				lane = maneuver.connectingLane
				wasOnLane = lane in state.lanes
				isOnLane = lane.intersects(car)
				if isOnLane and not wasOnLane:
					carState[car].lanes.add(lane)
					intersection_monitor.monitor.on_enterLane(timestamp, car, lane)
				elif wasOnLane and not isOnLane:
					carState[car].lanes.remove(lane)
					intersection_monitor.monitor.on_exitLane(timestamp, car, lane)
		wait

