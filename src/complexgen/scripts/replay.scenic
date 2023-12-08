""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('/home/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param replay_scenario = None
replay_scenario = globalParameters.replay_scenario
intersection = network.elements[replay_scenario.intersection_uid]
maneuver_uid = replay_scenario.maneuver_uid
blueprints = replay_scenario.blueprints
events = replay_scenario.events
curves = replay_scenario.curves
sim_trajs = replay_scenario.sim_trajectories
sample_size = int(replay_scenario.maxSteps)+1

from complexgen.core.utils import curves_to_trajectories
trajectory = curves_to_trajectories(curves, sim_trajs, sample_size)

import complexgen.simulators.carla.visualization as visualization
from complexgen.core.signals import SignalType

car2time2signal = {car:{e.frame:e.signal for e in es if e.name == 'signaledAtForkAtTime'} 
	for car, es in events.items()}

behavior ReplayBehavior():
	carla_world = simulation().world
	while True:
		t = simulation().currentTime
		state = trajectory[self.name][t]
		take SetPoseAction(state[0], state[1])

		if car2time2signal[self.name].__contains__(t):
			lights = SignalType[car2time2signal[self.name][t].upper()].to_vehicleLightState()
			take SetVehicleLightStateAction(lights)

		visualization.label_car(carla_world, self)

for carName, traj in trajectory.items():
	carState = traj[0]
	if not carName in {'ego', 'illegal'}:
		car = new Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with color Color(0, 0, 1),
			with behavior ReplayBehavior(),
			with physics False
	elif carName == 'ego':
		ego = new Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with color Color(0, 1, 0),
			with behavior ReplayBehavior(),
			with physics False

illegal = new Car ahead of ego by ego.length,
	with name 'illegal',
	with blueprint blueprints['ego'],
	with color Color(1, 0, 0),
	with behavior ReplayBehavior(),
	with physics False


monitor showIntersection():
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	wait

require monitor showIntersection()