""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('./maps/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param intersection = network.intersections[3] # unsignalized four-way intersection in Town05
intersection = globalParameters.intersection

param trajectories = None
trajectories = globalParameters.trajectories

import visualization
from intersection_monitor import CarState, SignalType
import carla

behavior ReplayBehavior():
	while True:
		state = trajectories[simulation().currentTime][self.name]
		take SetTransformAction(state[0], state[1])
		wait

cars = []
for carName, carState in trajectories[0].items(): 
	car = Car at carState[0], facing carState[1],
		with name carName,
		with behavior ReplayBehavior()
	cars.append(car)

ego = cars[0]






