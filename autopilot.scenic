""" Scenario Description
Ego-vehicle driven by Carla's autopilot.
All nonegos' behaviors are predetermined.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_uid = None
intersection = network.elements[globalParameters.intersection_uid]

param maneuver_uid = None
maneuver_uid = globalParameters.maneuver_uid

param trajectory = None
trajectory = globalParameters.trajectory
blueprints = globalParameters.blueprints

param event_monitor = None
event_monitor = globalParameters.event_monitor

param stop_speed_threshold = 0.01  # meters/seconds
stop_speed_threshold = globalParameters.stop_speed_threshold

import visualization
from signals import vehicleLightState_from_maneuverType

ARRIVAL_DISTANCE = 4 # meters

behavior SignalBehavior():
	l0_uid, l1_uid, l2_uid = maneuver_uid[self.name]
	l0 = network.elements[l0_uid]
	l1 = network.elements[l1_uid]
	l2 = network.elements[l2_uid]
	maneuverType = ManeuverType.guessTypeFromLanes(l0, l2, l1)
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

from agents.navigation.behavior_agent import BehaviorAgent
from scenic.simulators.carla.utils.utils import scenicToCarlaLocation

behavior CarlaBehaviorAgent():
	do SignalBehavior()
	take SetAutopilotAction(True)
	agent = BehaviorAgent(self.carlaActor, behavior='normal')
	carla_world = simulation().world
	src = scenicToCarlaLocation(trajectory[0][self.name][0], world=carla_world)
	dest = scenicToCarlaLocation(trajectory[-1][self.name][0], world=carla_world)
	agent.set_destination(src, dest, clean=True)
	agent.update_information()
	while agent.incoming_waypoint:
		control = agent.run_step()
		self.carlaActor.apply_control(control)
		wait
		agent.update_information()

for carName, carState in trajectory[0].items():
	if not carName in {'ego', 'illegal'}:
		car = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior ReplayBehavior(),
			with physics False
	elif carName == 'ego':
		ego = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with behavior CarlaBehaviorAgent()

from signals import SignalType
l0_uid, l1_uid, l2_uid = maneuver_uid['ego']
l0 = network.elements[l0_uid]
l1 = network.elements[l1_uid]
l2 = network.elements[l2_uid]
maneuverType = ManeuverType.guessTypeFromLanes(l0, l2, l1)
signal = SignalType.from_maneuverType(maneuverType)

monitor egoEvents:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	maneuvers = intersection.maneuvers
	arrived = False
	stopped = False
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
		if arrived and (not entered) and (not stopped) and ego.speed <= stop_speed_threshold:
			stopped = True
			# if ego.lane has a stop sign:
			event_monitor.on_stop(currentTime, 'ego', ego.lane.uid)

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