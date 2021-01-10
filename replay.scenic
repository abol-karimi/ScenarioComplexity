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
sim_trajectory = sim_result.trajectory
blueprints = globalParameters.blueprints

param vehicleLightStates = None
vehicleLightStates = globalParameters.vehicleLightStates

import visualization

behavior ReplayBehavior():
	lights = vehicleLightStates[self.name]
	take SetVehicleLightStateAction(lights)
	carla_world = simulation().world

	while True:
		currentTime = simulation().currentTime
		state = sim_trajectory[currentTime][self.name]
		take SetTransformAction(state[0], state[1])
		visualization.label_car(carla_world, self)
		wait

cars = []
for carName, carState in sim_trajectory[0].items(): 
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