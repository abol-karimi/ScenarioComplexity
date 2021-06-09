""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param replay_scenario = None
replay_scenario = globalParameters.replay_scenario
intersection = network.elements[replay_scenario.intersection_uid]
trajectory = replay_scenario.trajectory
blueprints = replay_scenario.blueprints

param event_monitor = None
event_monitor = globalParameters.event_monitor

import visualization

ARRIVAL_DISTANCE = 4 # meters

behavior ReplayBehavior():
	carla_world = simulation().world
	while True:
		t = simulation().currentTime
		state = trajectory[t][self.name]
		take SetTransformAction(state[0], state[1])

cars = []

for carName, carState in trajectory[0].items():
	if not carName in {'ego', 'illegal'}:
		car = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior ReplayBehavior(),
			with physics False
		cars.append(car)
	elif carName == 'ego':
		ego = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior ReplayBehavior(),
			with physics False
		cars.append(ego)			

illegal = Car ahead of ego by ego.length,
	with name 'illegal',
	with blueprint blueprints['ego'],
	with color Color(1, 0, 0),
	with behavior ReplayBehavior(),
	with physics False
cars.append(illegal)

monitor events:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	maneuvers = intersection.maneuvers
	arrived = {car: False for car in cars}
	entered = {car: False for car in cars}
	exited = {car: False for car in cars}
	lanes = {car: set() for car in cars}
	inIntersection = {car: False for car in cars}
	while True:
		currentTime = simulation().currentTime
		for car in cars:
			inIntersection[car] = intersection.intersects(car)
			
			if (not arrived[car]) and (distance from (front of car) to intersection) < ARRIVAL_DISTANCE:
				arrived[car] = True
				event_monitor.on_arrival(car.name, car.lane.uid, None, currentTime)
			if inIntersection[car] and not entered[car]:
				entered[car] = True
				event_monitor.on_entrance(car.name, car.lane.uid, currentTime)
			if entered[car] and (not exited[car]) and not inIntersection[car]:
				exited[car] = True
				event_monitor.on_exit(car.name, car.lane.uid, currentTime)

			for maneuver in maneuvers:
				lane = maneuver.connectingLane
				wasOnLane = lane in lanes[car]
				isOnLane = lane.intersects(car)
				if isOnLane and not wasOnLane:
					lanes[car].add(lane)
					event_monitor.on_enterLane(car.name, lane.uid, currentTime)
				elif wasOnLane and not isOnLane:
					lanes[car].remove(lane)
					event_monitor.on_exitLane(car.name, lane.uid, currentTime)
		wait