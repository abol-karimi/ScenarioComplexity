""" Scenario Description
Ego-vehicle arrives at an intersection.
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

import visualization
from signals import vehicleLightState_from_maneuverType

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

cars = []
for carName, carState in trajectory[0].items(): 
	car = Car at carState[0], facing carState[1],
		with name carName,
		with blueprint blueprints[carName],
		with behavior ReplayBehavior(),
		with physics False
	cars.append(car)

ego = cars[0]

monitor showIntersection:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	wait