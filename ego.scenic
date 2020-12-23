""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection = network.intersections[3] # unsignalized four-way intersection in Town05
intersection = globalParameters.intersection

param sim_result = None
sim_result = globalParameters.sim_result
sim_trajectory = None
sim_actions = None
if sim_result:
	sim_trajectory = sim_result.trajectory
	sim_actions = sim_result.actions

param blueprints = {'ego': 'vehicle.tesla.model3'}
blueprints = globalParameters.blueprints
print(blueprints)

import intersection_monitor
intersection_monitor.monitor.set_intersection(intersection)

import visualization
from intersection_monitor import CarState, SignalType
import carla
from signals import vehicleLightState_from_maneuverType, signalType_from_vehicleLightState

turnSignal = {}
if sim_trajectory:
	for actingCar, actions in sim_actions[0].items():
		if actingCar.name == 'ego':
			continue
		setVehicleLightStateAction = [a for a in actions
																	if isinstance(a, SetVehicleLightStateAction)][0]
		vehicleLightState = setVehicleLightStateAction.vehicleLightState
		turnSignal[actingCar.name] = signalType_from_vehicleLightState(vehicleLightState)
		
behavior ReplayBehavior():
	actingCars = sim_actions[0].keys()
	thisCar = [ac for ac in actingCars if ac.name == self.name][0]
	actions = sim_actions[0][thisCar]
	setVehicleLightStateAction = [a for a in actions if isinstance(a, SetVehicleLightStateAction)][0]
	take setVehicleLightStateAction

	while True:
		currentTime = simulation().currentTime
		state = sim_trajectory[currentTime][self.name]
		take SetTransformAction(state[0], state[1])
		wait

#CONSTANTS
EGO_SPEED = 4
ARRIVAL_DISTANCE = 4 # meters
SPAWN_DISTANCE = 20 # meters

behavior SignalBehavior(trajectory):
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
	lights = vehicleLightState_from_maneuverType(maneuverType)
	take SetVehicleLightStateAction(lights)

behavior LegalBehavior(target_speed, trajectory):
	do SignalBehavior(trajectory)
	do FollowTrajectoryBehavior(target_speed, trajectory)

#Ego vehicle
ego_maneuver = intersection.maneuvers[0]
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
ego = Car following roadDirection from ego_maneuver.startLane.centerline[-1] for -SPAWN_DISTANCE,
	with name 'ego',
	with blueprint blueprints['ego'],
	with behavior LegalBehavior(EGO_SPEED, ego_trajectory)
turnSignal['ego'] = SignalType.from_maneuver(ego_maneuver)

cars = [ego]
if sim_trajectory:
	for carName, carState in sim_trajectory[0].items():
		if carName == 'ego':
			continue
		car = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior ReplayBehavior()
		cars.append(car)
		
monitor carEvents:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection)
	maneuvers = intersection.maneuvers
	carState = {car:CarState() for car in cars}
	while True:
		timestamp = simulation().currentTime
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

