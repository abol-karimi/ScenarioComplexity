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
events = replay_scenario.events

import visualization
from signals import SignalType

car2time2signal = {car:{e.frame:e.signal for e in es if e.name == 'signaledAtForkAtTime'} 
	for car, es in events.items()}

behavior ReplayBehavior():
	carla_world = simulation().world
	while True:
		t = simulation().currentTime
		state = trajectory[t][self.name]
		take SetTransformAction(state[0], state[1])

		if t in car2time2signal[self.name]:
			lights = SignalType[car2time2signal[self.name][t].upper()].to_vehicleLightState()
			take SetVehicleLightStateAction(lights)

		visualization.label_car(carla_world, self)

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