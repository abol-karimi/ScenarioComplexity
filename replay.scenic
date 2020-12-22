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
sim_actions = sim_result.actions

import visualization
from intersection_monitor import CarState, SignalType
import carla

behavior ReplayBehavior():
	actingCars = sim_actions[0].keys()
	myCar = [ac for ac in actingCars if ac.name == self.name][0]
	actions = sim_actions[0][myCar]
	lightAction = [a for a in actions if isinstance(a, SetVehicleLightStateAction)][0]
	take lightAction

	while True:
		currentTime = simulation().currentTime
		state = sim_trajectory[currentTime][self.name]
		take SetTransformAction(state[0], state[1])
		wait

cars = []
for carName, carState in sim_trajectory[0].items(): 
	car = Car at carState[0], facing carState[1],
		with name carName,
		with behavior ReplayBehavior()
	cars.append(car)

ego = cars[0]






