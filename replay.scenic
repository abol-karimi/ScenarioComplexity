""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection_id = None
intersection = network.intersections[globalParameters.intersection_id]

param maneuver_id = None
maneuver_id = globalParameters.maneuver_id

param trajectory = None
trajectory = globalParameters.trajectory
blueprints = globalParameters.blueprints

import visualization
from signals import vehicleLightState_from_maneuverType

behavior SignalBehavior():
	maneuvers = intersection.maneuvers
	maneuver = maneuvers[maneuver_id[self.name]]
	trajectory = [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]
	maneuverType = ManeuverType.guessTypeFromLanes(trajectory[0], trajectory[2], trajectory[1])
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
	visualization.draw_intersection(carla_world, intersection)
	wait