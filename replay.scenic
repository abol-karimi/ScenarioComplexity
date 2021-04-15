""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param replay_scenario = None
replay_scenario = globalParameters.replay_scenario
intersection = network.elements[replay_scenario.intersection_uid]
maneuver_uid = replay_scenario.maneuver_uid
trajectory = replay_scenario.trajectory
blueprints = replay_scenario.blueprints

import visualization
from signals import vehicleLightState_from_maneuverType

behavior SignalBehavior():
	name = self.name if self.name != 'illegal' else 'ego'
	l0_uid, l1_uid, l2_uid = maneuver_uid[name]
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
			with behavior ReplayBehavior(),
			with physics False

illegal = Car ahead of ego by ego.length,
	with name 'illegal',
	with blueprint blueprints['ego'],
	with color Color(1, 0, 0),
	with behavior ReplayBehavior(),
	with physics False


monitor showIntersection:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	wait